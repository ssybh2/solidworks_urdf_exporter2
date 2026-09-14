"""Inventor web server extension for explicit Motor / Passive joint selection.

The exporter already treats top-level ``actuated_joints:`` in a CAD package's
``<robot>.joints.yaml`` as the authoritative MuJoCo actuator allow-list.  This
module exposes that existing setting to the browser without changing the CAD
kinematic graph or the closed-loop dependent/independent solver roles.

``sw2robot-web`` points here.  Everything except the two actuation endpoints is
delegated to :mod:`sw2robot.editor.inventor_webserver`.
"""
from __future__ import annotations

import json
import os
import re
import urllib.parse
import xml.etree.ElementTree as ET

from . import inventor_webserver as _inv
from . import webserver as _ws


def _config_path(pkg_dir, urdf_rel):
    """Return the joints.yaml used by the current package, creating no file."""
    if not pkg_dir or not urdf_rel:
        return None
    stem = os.path.splitext(os.path.basename(urdf_rel))[0]
    exact = os.path.join(str(pkg_dir), stem + ".joints.yaml")
    if os.path.isfile(exact):
        return exact
    try:
        candidates = sorted(
            os.path.join(str(pkg_dir), name)
            for name in os.listdir(str(pkg_dir))
            if name.endswith(".joints.yaml")
        )
    except OSError:
        return exact
    return candidates[0] if len(candidates) == 1 else exact


def _load_config(path):
    if not path or not os.path.isfile(path):
        return {}
    try:
        import yaml
        with open(path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except Exception:
        return {}
    return cfg if isinstance(cfg, dict) else {}


def _movable_joints(pkg_dir, urdf_rel):
    """Final joint names in the served working URDF, in URDF order."""
    if not pkg_dir or not urdf_rel:
        return []
    path = os.path.join(str(pkg_dir), str(urdf_rel).replace("/", os.sep))
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError):
        return []
    out = []
    for joint in root.findall("joint"):
        name = joint.get("name")
        jtype = (joint.get("type") or "fixed").lower()
        if name and jtype not in ("fixed", "floating", "planar"):
            out.append(name)
    return out


def _configured_actuated(cfg):
    """Configured final joint names, or None when explicit selection is absent."""
    if "actuated_joints" not in cfg:
        return None
    raw = cfg.get("actuated_joints")
    if not isinstance(raw, (list, tuple, set)):
        return set()
    overrides = cfg.get("joint_names")
    overrides = overrides if isinstance(overrides, dict) else {}
    from sw2robot.exporter.model import safe_name
    out = set()
    for value in raw:
        if value is None:
            continue
        name = str(value)
        out.add(safe_name(overrides.get(name, name)))
    return out


def actuation_payload(pkg_dir, urdf_rel):
    """Effective Motor / Passive state for the current CAD package.

    Before the first manual edit this mirrors the exporter's existing priority:
    explicit ``actuated_joints`` -> ``ACT_*`` convention -> automatic closed-loop
    independent/dependent selection.  The first toggle can therefore seed a
    custom list from the *current* effective state instead of unexpectedly
    turning every other motor off.
    """
    supported = bool(pkg_dir and urdf_rel and _ws._cad_mode(pkg_dir))
    movable = _movable_joints(pkg_dir, urdf_rel) if supported else []
    movable_set = set(movable)
    cfg_path = _config_path(pkg_dir, urdf_rel)
    cfg = _load_config(cfg_path)

    selected = _configured_actuated(cfg)
    if selected is not None:
        mode = "configured"
    else:
        prefixed = {name for name in movable if name.upper().startswith("ACT_")}
        if prefixed:
            selected = prefixed
            mode = "ACT_*"
        else:
            loop = _inv._loop_closure_payload(pkg_dir) if supported else {}
            independent = set(loop.get("independent") or [])
            dependent = set(loop.get("dependent") or [])
            # New sidecars define independent as all movable tree joints minus
            # dependent.  For older sidecars with no positive list, subtract
            # only the known dependents from all movable joints.
            selected = ((movable_set & independent) if independent
                        else (movable_set - dependent))
            mode = "auto"

    selected &= movable_set
    actuated = [name for name in movable if name in selected]
    return {
        "supported": supported,
        "mode": mode,
        "movable": movable,
        "actuated": actuated,
        "passive": [name for name in movable if name not in selected],
    }


def _strip_inline_actuated_list(text):
    """Normalize ``actuated_joints: [...]`` before the block-list helper runs."""
    return re.sub(
        r"(?m)^actuated_joints:\s*\[[^\n]*\]\s*(?:\n|$)", "", text)


def set_actuated_joint(pkg_dir, urdf_rel, joint, motor):
    """Persist one joint's Motor / Passive choice and return fresh status."""
    status = actuation_payload(pkg_dir, urdf_rel)
    if not status["supported"]:
        raise ValueError("Motor/Passive editing is available for CAD packages only")
    joint = str(joint or "").strip()
    if joint not in status["movable"]:
        raise ValueError(f"not a movable joint in the current URDF: {joint}")

    selected = set(status["actuated"])
    if bool(motor):
        selected.add(joint)
    else:
        selected.discard(joint)

    yml = _config_path(pkg_dir, urdf_rel)
    os.makedirs(os.path.dirname(yml), exist_ok=True)
    try:
        with open(yml, encoding="utf-8") as f:
            text = f.read()
    except FileNotFoundError:
        text = ""

    # Store stable pre-rename keys when possible.  actuator_fixes.py applies the
    # joint_names map at export time, so a later UI rename keeps the motor choice
    # attached to the same physical joint.
    inverse = _ws._names_inverse(text, "joint_names")
    ordered_final = [name for name in status["movable"] if name in selected]
    stored = [inverse.get(name, name) for name in ordered_final]

    # Preserve the rest of joints.yaml byte-for-byte as much as the upstream
    # list-block helper permits.  An explicit empty list is significant: it means
    # "zero motors" and must NOT collapse back to automatic mode.
    text = _strip_inline_actuated_list(text)
    text, _ = _ws._set_yaml_list_block(text, "actuated_joints", clear=True)
    if stored:
        text, _ = _ws._set_yaml_list_block(
            text, "actuated_joints", add=stored, append_if_absent=True)
    else:
        if text and not text.endswith("\n"):
            text += "\n"
        text += "actuated_joints: []\n"

    # Make the first creation undoable too: snapshot an empty config before
    # writing it, matching the rest of the CAD editor's YAML history model.
    if not os.path.exists(yml):
        open(yml, "a", encoding="utf-8").close()
    _ws._snapshot(
        str(pkg_dir), yml,
        f"{joint} -> {'Motor' if motor else 'Passive'}")
    with open(yml, "w", encoding="utf-8") as f:
        f.write(text)

    print(
        f"[sw2robot.web] actuation: {joint} -> "
        f"{'Motor' if motor else 'Passive'}; {len(selected)} motor(s)")
    return actuation_payload(pkg_dir, urdf_rel)


def _read_json(handler):
    try:
        n = int(handler.headers.get("Content-Length", "0") or 0)
        raw = handler.rfile.read(n) if n else b"{}"
        data = json.loads(raw.decode("utf-8"))
    except Exception as e:
        raise ValueError(f"invalid JSON body: {e}") from e
    if not isinstance(data, dict):
        raise ValueError("JSON body must be an object")
    return data


class _ActuationHandler(_inv._InventorHandler):
    """Inventor handler plus package-local actuation read/write endpoints."""

    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/api/actuation":
            cls = type(self)
            return self._send_json(
                actuation_payload(cls.pkg_dir, cls.urdf_rel))
        return super().do_GET()

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/api/set_actuated":
            cls = type(self)
            try:
                data = _read_json(self)
                joint = data.get("joint")
                motor = data.get("motor")
                if not isinstance(motor, bool):
                    raise ValueError("motor must be true or false")
                payload = set_actuated_joint(
                    cls.pkg_dir, cls.urdf_rel, joint, motor)
                return self._send_json(payload)
            except ValueError as e:
                return self._send_json({"error": str(e)}, 400)
            except OSError as e:
                return self._send_json({"error": str(e)}, 500)
        return super().do_POST()


def main():
    # inventor_webserver's extraction worker intentionally refers to its module
    # global _InventorHandler for root/package state.  Replace that global as
    # well as upstream's active handler so Inventor extraction keeps seeing the
    # same class attributes after this extra subclass is installed.
    _inv._InventorHandler = _ActuationHandler
    _ws._Handler = _ActuationHandler
    return _inv.main()


if __name__ == "__main__":
    main()
