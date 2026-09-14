"""Final MuJoCo actuator selection for CAD mechanisms.

A movable CAD joint is not necessarily actuated. Closed-loop linkages contain
passive revolute joints which must stay in the kinematic model but must not get a
MuJoCo actuator. ``exporter_fixes`` already removes loop joints classified as
IK-dependent; this layer adds an explicit, user-controlled final allow-list.

Selection priority:

1. top-level ``actuated_joints:`` in ``<robot>.joints.yaml``;
2. if no list exists, joint names beginning with ``ACT_`` (case-insensitive);
3. otherwise leave the previous automatic closed-loop result unchanged.

The explicit list may use pre-rename joint IDs; ``joint_names:`` is applied before
matching the final MJCF joint names. An explicit selection is also injected into
the closed-loop ``independent`` allow-list BEFORE the lower MJCF post-processor
runs, so a physically actuated loop joint cannot be deleted merely because the
automatic IK driver choice picked a different coordinate.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from functools import wraps


def _config_path(pkg_dir, robot_name):
    exact = os.path.join(pkg_dir, robot_name + ".joints.yaml")
    if os.path.isfile(exact):
        return exact
    try:
        candidates = sorted(
            os.path.join(pkg_dir, name) for name in os.listdir(pkg_dir)
            if name.endswith(".joints.yaml")
        )
    except OSError:
        return None
    return candidates[0] if len(candidates) == 1 else None


def _configured_actuated_joints(pkg_dir, robot_name):
    """Final-safe joint IDs from ``actuated_joints:``, or None when unspecified."""
    path = _config_path(pkg_dir, robot_name)
    if not path:
        return None
    try:
        import yaml

        with open(path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except Exception:
        return None
    if not isinstance(cfg, dict) or "actuated_joints" not in cfg:
        return None
    values = cfg.get("actuated_joints")
    if not isinstance(values, (list, tuple, set)):
        return None

    from .model import safe_name

    overrides = (cfg.get("joint_names")
                 if isinstance(cfg.get("joint_names"), dict) else {})
    result = set()
    for raw in values:
        if raw is None:
            continue
        name = str(raw)
        result.add(safe_name(overrides.get(name, name)))
    return result


def _working_urdf_act_prefix(pkg_dir, robot_name):
    """ACT_* joint IDs from the final working URDF, or None if convention unused."""
    path = os.path.join(pkg_dir, "urdf", robot_name + ".urdf")
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError):
        return None
    names = {
        joint.get("name") for joint in root.findall("joint")
        if joint.get("name") and joint.get("name").upper().startswith("ACT_")
    }
    return names or None


def _requested_actuated_joints(pkg_dir, robot_name):
    """Highest-priority pre-export actuator selection, or None for auto mode."""
    configured = _configured_actuated_joints(pkg_dir, robot_name)
    if configured is not None:
        return configured
    return _working_urdf_act_prefix(pkg_dir, robot_name)


def _prefix_actuated_joints(root):
    """ACT_* MJCF joint names, or None when that convention is not in use."""
    names = {
        joint.get("name") for joint in root.iter("joint")
        if joint.get("name") and joint.get("name").upper().startswith("ACT_")
    }
    return names or None


def _load_loop_cfg(pkg_dir, explicit=None):
    """Closed-loop sidecar used only to override its actuator driver choice."""
    if explicit is not None:
        return explicit if isinstance(explicit, dict) else None
    path = os.path.join(pkg_dir, "loop_closures.yaml")
    if not os.path.isfile(path):
        return None
    try:
        import yaml
        with open(path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or None
    except Exception:
        return None
    return cfg if isinstance(cfg, dict) and cfg.get("closures") else None


def _loop_cfg_with_actuators(pkg_dir, explicit, selected):
    """Copy closure config and make the explicit actuator set authoritative.

    The inner closed-loop MJCF fix uses ``independent`` as a positive actuator
    allow-list. Replacing it here happens before that layer runs, so explicit
    physical motors can override an arbitrary automatic IK driver selection.
    Geometry/equality closure data is unchanged.
    """
    if selected is None:
        return explicit
    cfg = _load_loop_cfg(pkg_dir, explicit)
    if cfg is None:
        return explicit
    out = dict(cfg)
    out["independent"] = sorted(set(selected))
    out["dependent"] = [
        name for name in (cfg.get("dependent") or []) if name not in selected
    ]
    return out


def _prune_actuators(root, selected=None):
    """Remove MJCF actuators whose joint is not in ``selected``.

    With ``selected=None`` use the ACT_ naming convention if present; with no
    convention either, do nothing. Returns a report used by tests and README
    diagnostics.
    """
    actuator = root.find("actuator")
    if actuator is None:
        return {"before": 0, "after": 0, "selected": [], "mode": "none"}
    before = len(list(actuator))
    mode = "configured" if selected is not None else None
    if selected is None:
        selected = _prefix_actuated_joints(root)
        if selected is not None:
            mode = "ACT_*"
    if selected is None:
        return {"before": before, "after": before, "selected": [], "mode": "auto"}
    selected = set(selected)
    for elem in list(actuator):
        joint = elem.get("joint")
        if joint and joint not in selected:
            actuator.remove(elem)
    after = len(list(actuator))
    active = [e.get("joint") for e in actuator if e.get("joint")]
    if after == 0:
        root.remove(actuator)
    return {"before": before, "after": after, "selected": active, "mode": mode}


def _readme_with_report(text, report):
    """Append the final actuator count after all MJCF post-processing layers."""
    if not report or report.get("mode") in ("auto", "none"):
        return text
    marker = "## Final actuator selection"
    if marker in text:
        return text
    active = ", ".join(report.get("selected") or []) or "(none)"
    return (text.rstrip() + "\n\n" + marker + "\n\n"
            + f"- Selection mode: `{report['mode']}`.\n"
            + f"- Final MJCF actuators: {report['after']} of "
              f"{report['before']} remaining movable-joint actuators.\n"
            + f"- Active joints: {active}.\n")


def _postprocess_path(out_root, pkg_dir, robot_name, selected=None):
    if selected is None:
        selected = _requested_actuated_joints(pkg_dir, robot_name)
    mjcf_dir = os.path.join(out_root, "mjcf")
    if not os.path.isdir(mjcf_dir):
        return None
    xmls = sorted(
        os.path.join(mjcf_dir, name) for name in os.listdir(mjcf_dir)
        if name.lower().endswith(".xml")
    )
    if not xmls:
        return None
    tree = ET.parse(xmls[0])
    report = _prune_actuators(tree.getroot(), selected)
    if report["before"] != report["after"]:
        ET.indent(tree, space="  ")
        tree.write(xmls[0], encoding="unicode", xml_declaration=False)

    readme = os.path.join(out_root, "README.md")
    if os.path.isfile(readme) and report.get("mode") not in ("auto", "none"):
        try:
            with open(readme, encoding="utf-8") as f:
                text = f.read()
            with open(readme, "w", encoding="utf-8") as f:
                f.write(_readme_with_report(text, report))
        except OSError:
            pass
    return report


def _postprocess_files(files, pkg_dir, robot_name, selected=None):
    if selected is None:
        selected = _requested_actuated_joints(pkg_dir, robot_name)
    out = []
    report = None
    readme_index = None
    for idx, (arc, data) in enumerate(files):
        if arc.lower().endswith(".xml") and "/mjcf/" in arc.replace("\\", "/"):
            root = ET.fromstring(data)
            report = _prune_actuators(root, selected)
            if report["before"] != report["after"]:
                ET.indent(root, space="  ")
                data = ET.tostring(root, encoding="utf-8")
        if arc.lower().endswith("/readme.md"):
            readme_index = idx
        out.append((arc, data))

    if (report is not None and readme_index is not None
            and report.get("mode") not in ("auto", "none")):
        arc, data = out[readme_index]
        try:
            text = data.decode("utf-8")
            out[readme_index] = (
                arc, _readme_with_report(text, report).encode("utf-8"))
        except UnicodeDecodeError:
            pass
    return out


def install():
    """Patch the final MJCF public entry points once."""
    from . import mjcf_export as mod

    if getattr(mod, "_cad_actuator_selection_installed", False):
        return
    original_write = mod.write_mjcf_package
    original_build = mod.build_mjcf_package

    @wraps(original_write)
    def write_mjcf_package(*args, **kwargs):
        pkg_dir = args[0] if args else kwargs["pkg_dir"]
        robot_name = args[1] if len(args) > 1 else kwargs["robot_name"]
        selected = _requested_actuated_joints(pkg_dir, robot_name)
        inner = dict(kwargs)
        override = _loop_cfg_with_actuators(
            pkg_dir, inner.get("loop_closures"), selected)
        if override is not None:
            inner["loop_closures"] = override
        out_root = original_write(*args, **inner)
        _postprocess_path(out_root, pkg_dir, robot_name, selected)
        return out_root

    @wraps(original_build)
    def build_mjcf_package(*args, **kwargs):
        pkg_dir = args[0] if args else kwargs["pkg_dir"]
        robot_name = args[1] if len(args) > 1 else kwargs["robot_name"]
        selected = _requested_actuated_joints(pkg_dir, robot_name)
        inner = dict(kwargs)
        override = _loop_cfg_with_actuators(
            pkg_dir, inner.get("loop_closures"), selected)
        if override is not None:
            inner["loop_closures"] = override
        pkg, files = original_build(*args, **inner)
        return pkg, _postprocess_files(files, pkg_dir, robot_name, selected)

    mod.write_mjcf_package = write_mjcf_package
    mod.build_mjcf_package = build_mjcf_package
    mod._cad_actuator_selection_installed = True
