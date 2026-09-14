"""Validation-safe defaults for CAD -> MuJoCo exports.

The CAD editor uses Motor / Passive as *interface metadata*: a Motor joint keeps
an actuator so a controller can drive it later, but a freshly exported model is
also used to validate mechanics with no controller attached.  For that use case
we deliberately default to:

* no derived joint damping (``backemf_damping=False``),
* no synthetic foot contact spheres (``foot_contacts=False``), and
* strict source-mesh collision geometry (``collision='copy'``).

Damping remains a code-level interface.  An advanced caller can explicitly pass
``backemf_damping=True`` to restore the converter's effort/velocity-derived
joint damping, or ``motor_damping=<scalar>`` / ``{joint: value}`` to assign a
specific damping to the final Motor joints after actuator pruning.

Synthetic foot contacts remain opt-in with ``foot_contacts=True``.  Approximate
collision modes remain available only when a caller deliberately passes
``strict_mesh_collision=False`` together with e.g. ``collision='coacd'``.  The
normal Web/CLI MJCF path therefore cannot silently turn CAD meshes into boxes,
hulls or CoACD parts because of a stale collision selector.

Floating-base spawn height is package metadata.  ``mujoco_spawn_height`` in
``<robot>.joints.yaml`` is an absolute world-Z value in metres.  If omitted, the
converter's automatically measured ``home`` base height is copied into the free
body's default pose, so ``mujoco.viewer.launch_from_path`` starts from the same
safe height as the home keyframe instead of from z=0.  A configured value
replaces both the default free-body height and the home keyframe height.
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from functools import wraps


def _prepare_kwargs(kwargs):
    """Return ``(kwargs_for_mjcf_export, motor_damping, strip_motor_damping)``.

    ``foot_contacts=None`` historically meant "add them for floating-base
    robots", so normalise None to False.  ``strict_mesh_collision`` is consumed
    here (the underlying exporter does not know that policy flag).
    """
    out = dict(kwargs)
    explicit_backemf = bool(out.get("backemf_damping", False))
    out.setdefault("backemf_damping", False)

    if out.get("foot_contacts") is None:
        out["foot_contacts"] = False

    strict_mesh = bool(out.pop("strict_mesh_collision", True))
    if strict_mesh:
        # 'copy' reuses the CAD/visual mesh as the collision STL without any
        # primitive fitting, convex hull or CoACD approximation.
        out["collision"] = "copy"
    else:
        out.setdefault("collision", "copy")

    motor_damping = out.pop("motor_damping", None)
    return out, motor_damping, not explicit_backemf


def _motor_joints(root):
    actuator = root.find("actuator")
    if actuator is None:
        return set()
    return {
        elem.get("joint") for elem in actuator
        if elem.get("joint")
    }


def _format_damping(value):
    value = float(value)
    if value < 0.0:
        raise ValueError("motor damping must be >= 0")
    return f"{value:.12g}"


def _apply_motor_damping(root, motor_damping=None, *, strip_default=True):
    """Apply the final Motor-joint damping policy to an MJCF tree.

    The final actuator list is authoritative because actuator_fixes has already
    pruned Passive joints before this outer wrapper runs.
    """
    motors = _motor_joints(root)
    if not motors:
        return False

    changed = False
    if motor_damping is None:
        if not strip_default:
            return False
        for joint in root.iter("joint"):
            if joint.get("name") in motors and "damping" in joint.attrib:
                del joint.attrib["damping"]
                changed = True
        return changed

    if isinstance(motor_damping, dict):
        values = {str(k): _format_damping(v) for k, v in motor_damping.items()}
    else:
        value = _format_damping(motor_damping)
        values = {name: value for name in motors}

    for joint in root.iter("joint"):
        name = joint.get("name")
        if name not in motors or name not in values:
            continue
        value = values[name]
        if joint.get("damping") != value:
            joint.set("damping", value)
            changed = True
    return changed


def _configured_spawn_height(pkg_dir, robot_name):
    """Configured absolute base Z in metres, or None for automatic clearance."""
    from .actuator_fixes import _load_config

    cfg = _load_config(pkg_dir, robot_name)
    if "mujoco_spawn_height" not in cfg:
        return None
    raw = cfg.get("mujoco_spawn_height")
    if raw is None:
        return None
    if isinstance(raw, str) and raw.strip().lower() in {"", "auto", "none", "null"}:
        return None
    if isinstance(raw, bool):
        raise ValueError("mujoco_spawn_height must be a number in metres or auto")
    try:
        value = float(raw)
    except (TypeError, ValueError) as e:
        raise ValueError(
            "mujoco_spawn_height must be a number in metres or auto") from e
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("mujoco_spawn_height must be finite and >= 0 metres")
    return value


def _fmt(value):
    return f"{float(value):.12g}"


def _apply_spawn_height(root, configured_height=None):
    """Make a floating base's default pose and ``home`` height agree.

    ``configured_height=None`` means automatic: take the Z already computed by
    the converter for the ``home`` keyframe and copy it into the free body's
    default pose.  A numeric value is an explicit absolute world-Z and is also
    written back to ``home``.  Returns ``(changed, resolved_height)``; fixed-base
    models return ``(False, None)``.
    """
    worldbody = root.find("worldbody")
    if worldbody is None:
        return False, None

    base = next(
        (body for body in worldbody.findall("body")
         if body.find("freejoint") is not None),
        None,
    )
    if base is None:
        return False, None

    home = None
    keyframe = root.find("keyframe")
    if keyframe is not None:
        home = next(
            (key for key in keyframe.findall("key")
             if key.get("name") == "home" and key.get("qpos")),
            None,
        )

    home_qpos = home.get("qpos").split() if home is not None else []
    if configured_height is None:
        if len(home_qpos) < 3:
            # No converter-computed safe height to copy.  Leave the default
            # pose untouched rather than inventing a number.
            return False, None
        try:
            height = float(home_qpos[2])
        except ValueError:
            return False, None
        if not math.isfinite(height):
            return False, None
    else:
        height = float(configured_height)
        if not math.isfinite(height) or height < 0.0:
            raise ValueError("spawn height must be finite and >= 0 metres")

    changed = False
    pos = (base.get("pos") or "0 0 0").split()
    while len(pos) < 3:
        pos.append("0")
    target = _fmt(height)
    if pos[2] != target:
        pos[2] = target
        base.set("pos", " ".join(pos[:3]))
        changed = True

    # A custom value is authoritative for Reset/Home too.  Automatic mode
    # leaves the converter's keyframe untouched and merely makes qpos0 match it.
    if configured_height is not None and len(home_qpos) >= 3:
        if home_qpos[2] != target:
            home_qpos[2] = target
            home.set("qpos", " ".join(home_qpos))
            changed = True

    return changed, height


def _postprocess_path(out_root, pkg_dir, robot_name,
                      motor_damping, strip_default):
    mjcf_dir = os.path.join(out_root, "mjcf")
    if not os.path.isdir(mjcf_dir):
        return
    spawn_height = _configured_spawn_height(pkg_dir, robot_name)
    for name in sorted(os.listdir(mjcf_dir)):
        if not name.lower().endswith(".xml"):
            continue
        path = os.path.join(mjcf_dir, name)
        tree = ET.parse(path)
        root = tree.getroot()
        changed = _apply_motor_damping(
            root, motor_damping, strip_default=strip_default)
        spawn_changed, _ = _apply_spawn_height(root, spawn_height)
        if changed or spawn_changed:
            ET.indent(tree, space="  ")
            tree.write(path, encoding="unicode", xml_declaration=False)
        return


def _postprocess_files(files, pkg_dir, robot_name,
                       motor_damping, strip_default):
    spawn_height = _configured_spawn_height(pkg_dir, robot_name)
    out = []
    for arc, data in files:
        norm = arc.replace("\\", "/").lower()
        if norm.endswith(".xml") and "/mjcf/" in norm:
            root = ET.fromstring(data)
            changed = _apply_motor_damping(
                root, motor_damping, strip_default=strip_default)
            spawn_changed, _ = _apply_spawn_height(root, spawn_height)
            if changed or spawn_changed:
                ET.indent(root, space="  ")
                data = ET.tostring(root, encoding="utf-8")
        out.append((arc, data))
    return out


def install():
    """Install the validation defaults around the final public MJCF entry points."""
    from . import mjcf_export as mod

    if getattr(mod, "_validation_defaults_installed", False):
        return

    original_write = mod.write_mjcf_package
    original_build = mod.build_mjcf_package

    @wraps(original_write)
    def write_mjcf_package(*args, **kwargs):
        pkg_dir = args[0] if args else kwargs["pkg_dir"]
        robot_name = args[1] if len(args) > 1 else kwargs["robot_name"]
        inner, motor_damping, strip_default = _prepare_kwargs(kwargs)
        out_root = original_write(*args, **inner)
        _postprocess_path(
            out_root, pkg_dir, robot_name, motor_damping, strip_default)
        return out_root

    @wraps(original_build)
    def build_mjcf_package(*args, **kwargs):
        pkg_dir = args[0] if args else kwargs["pkg_dir"]
        robot_name = args[1] if len(args) > 1 else kwargs["robot_name"]
        inner, motor_damping, strip_default = _prepare_kwargs(kwargs)
        pkg, files = original_build(*args, **inner)
        return pkg, _postprocess_files(
            files, pkg_dir, robot_name, motor_damping, strip_default)

    mod.write_mjcf_package = write_mjcf_package
    mod.build_mjcf_package = build_mjcf_package
    mod._validation_defaults_installed = True
