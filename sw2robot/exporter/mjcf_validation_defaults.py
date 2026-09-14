"""Validation-safe defaults for CAD -> MuJoCo exports.

The CAD editor uses Motor / Passive as *interface metadata*: a Motor joint keeps
an actuator so a controller can drive it later, but a freshly exported model is
also used to validate mechanics with no controller attached.  For that use case
we deliberately default to:

* no derived joint damping (``backemf_damping=False``),
* no synthetic foot contact spheres (``foot_contacts=False``),
* CAD-faithful collision using fine CoACD convex decomposition, and
* selective robot self-collision, including directly joint-connected bodies.

A single MuJoCo ``type=\"mesh\"`` geom is NOT an exact triangle-mesh collider:
MuJoCo collides it through its convex hull.  Therefore the normal Web/CLI MJCF
path decomposes each CAD collision mesh into a union of many convex mesh geoms
(``collision='coacd'``, ``coacd_quality='fine'``).  This preserves concavities
far better than one convex hull and never substitutes boxes/spheres/cylinders.

Self-collision also needs special handling.  MuJoCo filters direct parent-child
body collisions by default, which lets adjacent links pass through one another.
The default policy disables that parent filter and enables robot-vs-robot contact
for all collision geoms.  Only body pairs tied together by the restored CAD
closed-loop equality hinges are explicitly excluded, because those equality
constraints already define the mechanical connection and simultaneous contact
there can overconstrain the loop.

Damping remains a code-level interface.  An advanced caller can explicitly pass
``backemf_damping=True`` to restore the converter's effort/velocity-derived
joint damping, or ``motor_damping=<scalar>`` / ``{joint: value}`` to assign a
specific damping to the final Motor joints after actuator pruning.

``strict_mesh_collision`` is retained as a compatibility policy flag.  Its
default ``True`` now means the CAD-faithful fine-CoACD path described above.
Passing ``strict_mesh_collision=False`` restores the caller-controlled collision
mode (``copy``, ``hull``, ``coacd``, primitive modes, ...).

The legacy ``self_collision`` bool remains available to code callers: explicitly
passing it preserves the converter's old full-on/full-off behaviour.  The new
``selective_self_collision`` policy is the default only when ``self_collision``
is not supplied; it can also be explicitly enabled/disabled by advanced callers.

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

_COLLISION_GROUP = "3"
_ROBOT_CONTYPE = "2"
# bit 0 = environment (ground), bit 1 = robot.  Robot geoms advertise affinity
# to BOTH so they collide with the ground and with other robot bodies.
_ROBOT_CONAFFINITY_SELF = "3"


def _prepare_kwargs(kwargs):
    """Return exporter kwargs plus post-process policy values.

    Returns ``(kwargs_for_mjcf_export, motor_damping, strip_motor_damping,
    selective_self_collision)``.

    ``foot_contacts=None`` historically meant "add them for floating-base
    robots", so normalise None to False.  ``strict_mesh_collision`` and
    ``selective_self_collision`` are wrapper policy flags; the underlying
    converter does not know them.

    Compatibility rule: if a caller explicitly passes the legacy
    ``self_collision`` bool, preserve that old full-on/full-off request unless
    ``selective_self_collision`` is explicitly supplied too.  Web/normal calls
    do not pass ``self_collision``, so they receive the safer selective mode.
    """
    out = dict(kwargs)
    explicit_backemf = bool(out.get("backemf_damping", False))
    out.setdefault("backemf_damping", False)

    if out.get("foot_contacts") is None:
        out["foot_contacts"] = False

    strict_mesh = bool(out.pop("strict_mesh_collision", True))
    if strict_mesh:
        # MuJoCo convexifies each individual mesh geom.  Fine CoACD therefore
        # turns one concave CAD mesh into many convex mesh geoms whose UNION is
        # much closer to the original surface than a single convex hull.
        out["collision"] = "coacd"
        out["coacd_quality"] = "fine"
    else:
        out.setdefault("collision", "copy")

    selective_arg = out.pop("selective_self_collision", None)
    explicit_legacy_self = "self_collision" in out
    if selective_arg is None:
        selective_self_collision = not explicit_legacy_self
    else:
        selective_self_collision = bool(selective_arg)

    if selective_self_collision:
        # Ask the converter for its stable no-self-collision form, then enable
        # exactly the desired contacts below.  This keeps behaviour independent
        # of scikit-robot's blanket self-collision implementation.
        out["self_collision"] = False

    motor_damping = out.pop("motor_damping", None)
    return (
        out,
        motor_damping,
        not explicit_backemf,
        selective_self_collision,
    )


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


def _pair_key(body1, body2):
    if not body1 or not body2 or body1 == body2:
        return None
    return tuple(sorted((str(body1), str(body2))))


def _tree_connected_body_pairs(worldbody):
    """Direct MJCF parent-child body pairs after fixed-link merging."""
    pairs = set()

    def walk(container, parent_name=None):
        for body in container.findall("body"):
            name = body.get("name")
            key = _pair_key(parent_name, name)
            if key is not None:
                pairs.add(key)
            walk(body, name)

    walk(worldbody)
    return pairs


def _loop_connected_body_pairs(root):
    """Body pairs connected by restored CAD loop equality/connect constraints."""
    pairs = set()
    equality = root.find("equality")
    if equality is None:
        return pairs
    for connect in equality.findall("connect"):
        key = _pair_key(connect.get("body1"), connect.get("body2"))
        if key is not None:
            pairs.add(key)
    return pairs


def _ensure_option_flag(root):
    option = root.find("option")
    changed = False
    if option is None:
        option = ET.Element("option")
        children = list(root)
        compiler = root.find("compiler")
        idx = children.index(compiler) + 1 if compiler in children else 0
        root.insert(idx, option)
        changed = True
    flag = option.find("flag")
    if flag is None:
        flag = ET.SubElement(option, "flag")
        changed = True
    return flag, changed


def _disable_parent_collision_filter(root):
    """Allow collision checks between bodies in a direct parent-child relation."""
    flag, changed = _ensure_option_flag(root)
    if flag.get("filterparent") != "disable":
        flag.set("filterparent", "disable")
        changed = True
    return changed


def _ensure_contact(root):
    contact = root.find("contact")
    if contact is not None:
        return contact, False

    contact = ET.Element("contact")
    children = list(root)
    worldbody = root.find("worldbody")
    if worldbody is not None and worldbody in children:
        root.insert(children.index(worldbody) + 1, contact)
    else:
        idx = next(
            (i for i, child in enumerate(children)
             if child.tag in ("equality", "actuator", "sensor", "keyframe")),
            len(children),
        )
        root.insert(idx, contact)
    return contact, True


def _apply_selective_self_collision(root):
    """Enable robot self-collision, including adjacent tree links.

    Collision geoms emitted by the converter are group 3.  They keep contype bit
    1 (value 2) and receive conaffinity bits 0+1 (value 3):

    * robot vs ground (1/1) still collides via ``1 & 3``;
    * robot vs robot collides via ``2 & 3``.

    MuJoCo normally filters direct parent-child collisions before the bitmask
    test, so ``filterparent`` is explicitly disabled.  This is required for a
    hinge child to be physically stopped by the geometry of its parent instead
    of being allowed to rotate through it.

    We exclude only restored CAD loop-equality endpoint pairs.  Those bodies are
    already tied together by equality constraints; applying collision at the
    same virtual hinge can overconstrain the closed loop.  Ordinary tree-joint
    parent-child pairs are deliberately NOT excluded.

    Returns ``(changed, report)``.
    """
    worldbody = root.find("worldbody")
    if worldbody is None:
        return False, {
            "collision_geoms": 0,
            "excluded_pairs": [],
            "tree_pairs": [],
            "filterparent_disabled": False,
        }

    changed = _disable_parent_collision_filter(root)
    collision_geoms = 0
    for body in worldbody.iter("body"):
        for geom in body.findall("geom"):
            if geom.get("group") != _COLLISION_GROUP:
                continue
            collision_geoms += 1
            if geom.get("contype") != _ROBOT_CONTYPE:
                geom.set("contype", _ROBOT_CONTYPE)
                changed = True
            if geom.get("conaffinity") != _ROBOT_CONAFFINITY_SELF:
                geom.set("conaffinity", _ROBOT_CONAFFINITY_SELF)
                changed = True

    tree_pairs = _tree_connected_body_pairs(worldbody)
    excluded = _loop_connected_body_pairs(root)
    excluded = {pair for pair in excluded if pair is not None}

    if excluded:
        contact, created = _ensure_contact(root)
        changed = changed or created
        existing = {
            _pair_key(elem.get("body1"), elem.get("body2"))
            for elem in contact.findall("exclude")
        }
        existing.discard(None)
        for body1, body2 in sorted(excluded):
            if (body1, body2) in existing:
                continue
            ET.SubElement(contact, "exclude", {
                "body1": body1,
                "body2": body2,
            })
            existing.add((body1, body2))
            changed = True

    return changed, {
        "collision_geoms": collision_geoms,
        "excluded_pairs": sorted(excluded),
        "tree_pairs": sorted(tree_pairs),
        "filterparent_disabled": True,
    }


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
                      motor_damping, strip_default,
                      selective_self_collision):
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
        changed = changed or spawn_changed
        if selective_self_collision:
            collision_changed, _ = _apply_selective_self_collision(root)
            changed = changed or collision_changed
        if changed:
            ET.indent(tree, space="  ")
            tree.write(path, encoding="unicode", xml_declaration=False)
        return


def _postprocess_files(files, pkg_dir, robot_name,
                       motor_damping, strip_default,
                       selective_self_collision):
    spawn_height = _configured_spawn_height(pkg_dir, robot_name)
    out = []
    for arc, data in files:
        norm = arc.replace("\\", "/").lower()
        if norm.endswith(".xml") and "/mjcf/" in norm:
            root = ET.fromstring(data)
            changed = _apply_motor_damping(
                root, motor_damping, strip_default=strip_default)
            spawn_changed, _ = _apply_spawn_height(root, spawn_height)
            changed = changed or spawn_changed
            if selective_self_collision:
                collision_changed, _ = _apply_selective_self_collision(root)
                changed = changed or collision_changed
            if changed:
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
        (
            inner,
            motor_damping,
            strip_default,
            selective_self_collision,
        ) = _prepare_kwargs(kwargs)
        out_root = original_write(*args, **inner)
        _postprocess_path(
            out_root, pkg_dir, robot_name,
            motor_damping, strip_default, selective_self_collision)
        return out_root

    @wraps(original_build)
    def build_mjcf_package(*args, **kwargs):
        pkg_dir = args[0] if args else kwargs["pkg_dir"]
        robot_name = args[1] if len(args) > 1 else kwargs["robot_name"]
        (
            inner,
            motor_damping,
            strip_default,
            selective_self_collision,
        ) = _prepare_kwargs(kwargs)
        pkg, files = original_build(*args, **inner)
        return pkg, _postprocess_files(
            files, pkg_dir, robot_name,
            motor_damping, strip_default, selective_self_collision)

    mod.write_mjcf_package = write_mjcf_package
    mod.build_mjcf_package = build_mjcf_package
    mod._validation_defaults_installed = True
