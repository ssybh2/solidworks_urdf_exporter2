"""Inventor web server extension for explicit Motor / Passive joint selection.

The exporter already treats top-level ``actuated_joints:`` in a CAD package's
``<robot>.joints.yaml`` as the authoritative MuJoCo actuator allow-list.  This
module exposes that existing setting to the browser without changing the CAD
kinematic graph or the closed-loop dependent/independent solver roles.

It also exposes ``mujoco_actuation_enabled``.  Motor joints keep their actuator
interfaces regardless of this flag; the flag only decides whether MuJoCo lets
those actuators apply force immediately when the exported model is loaded.

``mujoco_spawn_height`` is an optional absolute world-Z height in metres for the
floating base.  Missing / null means Auto: the exporter uses the mesh-derived
safe ``home`` height.  A numeric value overrides both the default free-body pose
and the home keyframe, so direct MuJoCo viewer launches and Reset/Home agree.

``sw2robot-web`` points here.  Everything except the actuation endpoints is
delegated to :mod:`sw2robot.editor.inventor_webserver`.
"""
from __future__ import annotations

import json
import math
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


def _config_bool(cfg, key, default=False):
    raw = cfg.get(key, default)
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, str):
        return raw.strip().lower() in {"1", "true", "yes", "on", "enable", "enabled"}
    return bool(raw)


def _config_optional_nonnegative_float(cfg, key):
    if key not in cfg or cfg.get(key) is None:
        return None
    raw = cfg.get(key)
    if isinstance(raw, str) and raw.strip().lower() in {"", "auto", "none", "null"}:
        return None
    if isinstance(raw, bool):
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value >= 0.0 else None


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
    """Effective Motor / Passive state and MuJoCo startup export settings."""
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
        # Default False is intentional: Motor means "has an actuator interface";
        # it does not mean "energise it immediately when the model is loaded".
        "mujoco_actuation_enabled": _config_bool(
            cfg, "mujoco_actuation_enabled", False),
        # None means Auto: use the mesh-derived home clearance at export time.
        "mujoco_spawn_height": _config_optional_nonnegative_float(
            cfg, "mujoco_spawn_height"),
    }


def _strip_inline_actuated_list(text):
    """Normalize ``actuated_joints: [...]`` before the block-list helper runs."""
    return re.sub(
        r"(?m)^actuated_joints:\s*\[[^\n]*\]\s*(?:\n|$)", "", text)


def _set_top_level_bool(text, key, enabled):
    """Set one top-level YAML bool while leaving the rest of the file alone."""
    line = f"{key}: {'true' if enabled else 'false'}"
    pat = re.compile(rf"(?m)^{re.escape(key)}:\s*[^\n]*(?:\n|$)")
    if pat.search(text):
        return pat.sub(line + "\n", text, count=1)
    if text and not text.endswith("\n"):
        text += "\n"
    return text + line + "\n"


def _set_top_level_optional_number(text, key, value):
    """Set a top-level numeric scalar, or remove it entirely for Auto mode."""
    pat = re.compile(rf"(?m)^{re.escape(key)}:\s*[^\n]*(?:\n|$)")
    if value is None:
        return pat.sub("", text, count=1)
    line = f"{key}: {float(value):.12g}"
    if pat.search(text):
        return pat.sub(line + "\n", text, count=1)
    if text and not text.endswith("\n"):
        text += "\n"
    return text + line + "\n"


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

    inverse = _ws._names_inverse(text, "joint_names")
    ordered_final = [name for name in status["movable"] if name in selected]
    stored = [inverse.get(name, name) for name in ordered_final]

    text = _strip_inline_actuated_list(text)
    text, _ = _ws._set_yaml_list_block(text, "actuated_joints", clear=True)
    if stored:
        text, _ = _ws._set_yaml_list_block(
            text, "actuated_joints", add=stored, append_if_absent=True)
    else:
        if text and not text.endswith("\n"):
            text += "\n"
        text += "actuated_joints: []\n"

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


def set_mujoco_actuation_startup(pkg_dir, urdf_rel, enabled):
    """Persist whether exported MuJoCo actuators are energised at model load."""
    status = actuation_payload(pkg_dir, urdf_rel)
    if not status["supported"]:
        raise ValueError("MuJoCo actuation startup is available for CAD packages only")
    if not isinstance(enabled, bool):
        raise ValueError("enabled must be true or false")

    yml = _config_path(pkg_dir, urdf_rel)
    os.makedirs(os.path.dirname(yml), exist_ok=True)
    try:
        with open(yml, encoding="utf-8") as f:
            text = f.read()
    except FileNotFoundError:
        text = ""

    if not os.path.exists(yml):
        open(yml, "a", encoding="utf-8").close()
    _ws._snapshot(
        str(pkg_dir), yml,
        "MuJoCo actuation on startup -> " + ("enabled" if enabled else "disabled"))
    text = _set_top_level_bool(text, "mujoco_actuation_enabled", enabled)
    with open(yml, "w", encoding="utf-8") as f:
        f.write(text)

    print(
        "[sw2robot.web] MuJoCo actuation on startup -> "
        + ("enabled" if enabled else "disabled"))
    return actuation_payload(pkg_dir, urdf_rel)


def set_mujoco_spawn_height(pkg_dir, urdf_rel, height):
    """Persist absolute floating-base spawn Z, or clear it for Auto mode."""
    status = actuation_payload(pkg_dir, urdf_rel)
    if not status["supported"]:
        raise ValueError("MuJoCo spawn height is available for CAD packages only")

    if height is not None:
        if isinstance(height, bool):
            raise ValueError("height must be a number in metres or null for Auto")
        try:
            height = float(height)
        except (TypeError, ValueError) as e:
            raise ValueError(
                "height must be a number in metres or null for Auto") from e
        if not math.isfinite(height) or height < 0.0:
            raise ValueError("height must be finite and >= 0 metres")

    yml = _config_path(pkg_dir, urdf_rel)
    os.makedirs(os.path.dirname(yml), exist_ok=True)
    try:
        with open(yml, encoding="utf-8") as f:
            text = f.read()
    except FileNotFoundError:
        text = ""

    if not os.path.exists(yml):
        open(yml, "a", encoding="utf-8").close()
    label = "Auto" if height is None else f"{height:.6g} m"
    _ws._snapshot(str(pkg_dir), yml, f"MuJoCo spawn height -> {label}")
    text = _set_top_level_optional_number(text, "mujoco_spawn_height", height)
    with open(yml, "w", encoding="utf-8") as f:
        f.write(text)

    print(f"[sw2robot.web] MuJoCo spawn height -> {label}")
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
    """Inventor handler plus package-local MuJoCo metadata endpoints."""

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
        if path == "/api/set_actuation_startup":
            cls = type(self)
            try:
                data = _read_json(self)
                enabled = data.get("enabled")
                if not isinstance(enabled, bool):
                    raise ValueError("enabled must be true or false")
                payload = set_mujoco_actuation_startup(
                    cls.pkg_dir, cls.urdf_rel, enabled)
                return self._send_json(payload)
            except ValueError as e:
                return self._send_json({"error": str(e)}, 400)
            except OSError as e:
                return self._send_json({"error": str(e)}, 500)
        if path == "/api/set_spawn_height":
            cls = type(self)
            try:
                data = _read_json(self)
                payload = set_mujoco_spawn_height(
                    cls.pkg_dir, cls.urdf_rel, data.get("height"))
                return self._send_json(payload)
            except ValueError as e:
                return self._send_json({"error": str(e)}, 400)
            except OSError as e:
                return self._send_json({"error": str(e)}, 500)
        return super().do_POST()


def main():
    _inv._InventorHandler = _ActuationHandler
    _ws._Handler = _ActuationHandler
    return _inv.main()


if __name__ == "__main__":
    main()
