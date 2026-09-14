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
matching the final MJCF joint names.
"""

from __future__ import annotations

from functools import wraps
import os
import xml.etree.ElementTree as ET


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
    overrides = cfg.get("joint_names") if isinstance(cfg.get("joint_names"), dict) else {}
    result = set()
    for raw in values:
        if raw is None:
            continue
        name = str(raw)
        result.add(safe_name(overrides.get(name, name)))
    return result


def _prefix_actuated_joints(root):
    """ACT_* MJCF joint names, or None when that convention is not in use."""
    names = {
        joint.get("name") for joint in root.iter("joint")
        if joint.get("name") and joint.get("name").upper().startswith("ACT_")
    }
    return names or None


def _prune_actuators(root, selected=None):
    """Remove MJCF actuators whose joint is not in ``selected``.

    With ``selected=None`` use the ACT_ naming convention if present; with no
    convention either, do nothing. Returns a small report for tests/diagnostics.
    """
    actuator = root.find("actuator")
    if actuator is None:
        return {"before": 0, "after": 0, "selected": []}
    before = len(list(actuator))
    if selected is None:
        selected = _prefix_actuated_joints(root)
    if selected is None:
        return {"before": before, "after": before, "selected": []}
    selected = set(selected)
    for elem in list(actuator):
        joint = elem.get("joint")
        if joint and joint not in selected:
            actuator.remove(elem)
    after = len(list(actuator))
    active = [e.get("joint") for e in actuator if e.get("joint")]
    if after == 0:
        root.remove(actuator)
    return {"before": before, "after": after, "selected": active}


def _postprocess_path(out_root, pkg_dir, robot_name):
    selected = _configured_actuated_joints(pkg_dir, robot_name)
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
    return report


def _postprocess_files(files, pkg_dir, robot_name):
    selected = _configured_actuated_joints(pkg_dir, robot_name)
    out = []
    for arc, data in files:
        if arc.lower().endswith(".xml") and "/mjcf/" in arc.replace("\\", "/"):
            root = ET.fromstring(data)
            report = _prune_actuators(root, selected)
            if report["before"] != report["after"]:
                ET.indent(root, space="  ")
                data = ET.tostring(root, encoding="utf-8")
        out.append((arc, data))
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
        out_root = original_write(*args, **kwargs)
        _postprocess_path(out_root, pkg_dir, robot_name)
        return out_root

    @wraps(original_build)
    def build_mjcf_package(*args, **kwargs):
        pkg_dir = args[0] if args else kwargs["pkg_dir"]
        robot_name = args[1] if len(args) > 1 else kwargs["robot_name"]
        pkg, files = original_build(*args, **kwargs)
        return pkg, _postprocess_files(files, pkg_dir, robot_name)

    mod.write_mjcf_package = write_mjcf_package
    mod.build_mjcf_package = build_mjcf_package
    mod._cad_actuator_selection_installed = True
