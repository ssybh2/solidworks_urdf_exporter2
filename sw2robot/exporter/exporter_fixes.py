"""Post-process MJCF exports with CAD semantics that URDF cannot represent.

The working URDF is intentionally a tree. CAD assemblies can contain closed
kinematic loops, so ``build()`` persists the cut loop edges in
``loop_closures.yaml``. The stock URDF->MJCF converter cannot see that sidecar
and therefore used to export an open mechanism and attach an actuator to every
tree joint.

This module installs one compatibility layer around the two public MJCF export
entry points. It restores information deliberately lost by the URDF tree step:

* each dropped revolute edge becomes TWO MuJoCo ``equality/connect`` constraints
  on collinear witness points (MuJoCo's documented way to model a hinge outside
  the kinematic tree);
* only ``loop_closures.yaml: independent`` joints keep actuators, while passive
  ``dependent`` joints remain free coordinates solved by the equality system;
* pure-black visual geoms emitted by converters that lose CAD material colours
  receive a neutral visible fallback; explicit web ``colors:`` overrides still
  win.

Both the on-disk CLI export and the web in-memory ZIP export go through the same
post-process, so they cannot drift apart.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from functools import wraps

import numpy as np

_WITNESS_SEPARATION_M = 0.03
_DEFAULT_VISUAL_RGBA = "0.72 0.72 0.72 1"


def _load_loop_closures(pkg_dir, explicit=None):
    if explicit is not None:
        return explicit or None
    path = os.path.join(pkg_dir, "loop_closures.yaml")
    if not os.path.isfile(path):
        return None
    try:
        import yaml
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or None
    except Exception:
        return None
    if not isinstance(data, dict) or not data.get("closures"):
        return None
    return data


def _hex_rgba(value):
    if not value:
        return None
    s = str(value).strip().lstrip("#")
    if len(s) == 6:
        s += "ff"
    if len(s) != 8:
        return None
    try:
        c = [int(s[i:i + 2], 16) / 255.0 for i in (0, 2, 4, 6)]
    except ValueError:
        return None
    return " ".join(f"{v:.6g}" for v in c)


def _is_black_rgba(text):
    if not text:
        return False
    try:
        v = [float(x) for x in text.split()]
    except ValueError:
        return False
    return len(v) >= 3 and max(abs(v[0]), abs(v[1]), abs(v[2])) < 1e-8


def _working_urdf_parents(pkg_dir, robot_name):
    """Return ``child -> (parent, joint_type)`` from the unmerged working URDF."""
    path = os.path.join(pkg_dir, "urdf", robot_name + ".urdf")
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError):
        return {}
    out = {}
    for joint in root.findall("joint"):
        pe = joint.find("parent")
        ce = joint.find("child")
        if pe is None or ce is None:
            continue
        child = ce.get("link")
        parent = pe.get("link")
        if child and parent:
            out[child] = (parent, joint.get("type", "fixed"))
    return out


def _resolve_merged_body(name, body_names, parents):
    """Map a pre-merge URDF link name onto the surviving MuJoCo body."""
    cur = name
    seen = set()
    while cur and cur not in seen:
        if cur in body_names:
            return cur
        seen.add(cur)
        rec = parents.get(cur)
        if rec is None:
            break
        parent, joint_type = rec
        # merge_fixed_links only removes links across fixed joints. Never jump
        # across a movable edge merely to make a closure compile.
        if joint_type != "fixed":
            break
        cur = parent
    return None


def _ensure_equality_before_actuator(root):
    eq = root.find("equality")
    if eq is not None:
        return eq
    eq = ET.Element("equality")
    children = list(root)
    idx = next((i for i, child in enumerate(children)
                if child.tag in ("actuator", "sensor", "keyframe")), len(children))
    root.insert(idx, eq)
    return eq


def _add_closed_loop_equalities(root, cfg, pkg_dir, robot_name, mjcf_mod):
    """Restore dropped revolute loop edges as two point-connect constraints."""
    if not cfg or not cfg.get("closures"):
        return {"closures": 0, "connects": 0, "skipped": []}

    worldbody = root.find("worldbody")
    if worldbody is None:
        return {"closures": 0, "connects": 0, "skipped": ["no <worldbody>"]}

    frames = mjcf_mod._world_frames(worldbody, {})
    body_names = set(frames)
    parents = _working_urdf_parents(pkg_dir, robot_name)
    base = mjcf_mod._base_body(worldbody)
    base_name = "base_link" if "base_link" in frames else (
        base.get("name") if base is not None else None)
    if not base_name or base_name not in frames:
        return {"closures": 0, "connects": 0,
                "skipped": ["could not identify MJCF base body"]}
    base_R, base_p = frames[base_name]

    eq = _ensure_equality_before_actuator(root)
    existing = {e.get("name") for e in eq if e.get("name")}
    n_closure = 0
    n_connect = 0
    skipped = []

    for i, spec in enumerate(cfg.get("closures") or []):
        raw_a, raw_b = spec.get("link_a"), spec.get("link_b")
        body_a = _resolve_merged_body(raw_a, body_names, parents)
        body_b = _resolve_merged_body(raw_b, body_names, parents)
        if not body_a or not body_b:
            skipped.append(
                f"{raw_a}<->{raw_b}: endpoint missing after fixed-link merge")
            continue
        if body_a == body_b:
            # The whole closure collapsed into one rigid body; no equality is
            # needed and adding a same-body equality would only overconstrain it.
            continue

        try:
            p_base = np.asarray(spec["point"], dtype=float)
            axis_base = np.asarray(spec["axis"], dtype=float)
        except (KeyError, TypeError, ValueError):
            skipped.append(f"{raw_a}<->{raw_b}: invalid point/axis")
            continue
        if p_base.shape != (3,) or axis_base.shape != (3,):
            skipped.append(f"{raw_a}<->{raw_b}: point/axis is not a 3-vector")
            continue
        na = float(np.linalg.norm(axis_base))
        if na < 1e-12:
            skipped.append(f"{raw_a}<->{raw_b}: zero axis")
            continue
        axis_base /= na

        # loop_closures.yaml stores point/axis in base_link coordinates. MuJoCo
        # connect's body+anchor form wants the anchor in body1-local coordinates.
        p_world = base_p + base_R @ p_base
        axis_world = base_R @ axis_base
        body_R, body_p = frames[body_a]

        # MuJoCo documents that TWO point-connect constraints between the same
        # body pair model a hinge outside the kinematic tree. Their collinear
        # anchors remove the 3 translational + 2 axis-orientation DOFs while
        # leaving rotation about the common line free.
        for k, offset in enumerate((0.0, _WITNESS_SEPARATION_M)):
            world_anchor = p_world + offset * axis_world
            local_anchor = body_R.T @ (world_anchor - body_p)
            name = f"loop_{i}_{k}"
            suffix = 1
            while name in existing:
                suffix += 1
                name = f"loop_{i}_{k}_{suffix}"
            existing.add(name)
            ET.SubElement(eq, "connect", {
                "name": name,
                "body1": body_a,
                "body2": body_b,
                "anchor": " ".join(f"{float(v):.10g}" for v in local_anchor),
            })
            n_connect += 1
        n_closure += 1

    if not list(eq):
        root.remove(eq)
    return {"closures": n_closure, "connects": n_connect, "skipped": skipped}


def _filter_actuators(root, cfg):
    """Keep only CAD-independent joints actuated; dependent loop joints are passive."""
    actuator = root.find("actuator")
    if actuator is None:
        return {"before": 0, "after": 0, "active": []}
    before = len(list(actuator))
    if not cfg:
        return {"before": before, "after": before, "active": []}

    independent = set(cfg.get("independent") or [])
    dependent = set(cfg.get("dependent") or [])
    # _collect_loop_closures defines independent as ALL movable tree joints minus
    # the IK-solved dependents, including non-loop wheel joints. Prefer that
    # positive allow-list. Older sidecars may only have dependent.
    for elem in list(actuator):
        joint = elem.get("joint")
        if not joint:
            continue
        remove = (joint not in independent) if independent else (joint in dependent)
        if remove:
            actuator.remove(elem)
    after = len(list(actuator))
    active = [e.get("joint") for e in actuator if e.get("joint")]
    if after == 0:
        root.remove(actuator)
    return {"before": before, "after": after, "active": active}


def _fix_visual_colors(root, colors=None):
    """Apply explicit overrides and replace converter-produced pure black."""
    colors = colors or {}
    changed = 0
    for body in root.iter("body"):
        body_name = body.get("name") or ""
        body_color = _hex_rgba(colors.get(body_name))
        for geom in body.findall("geom"):
            if geom.get("group") != "2" or geom.get("contype") not in (None, "0"):
                continue
            mesh_name = geom.get("mesh") or ""
            rgba = body_color or _hex_rgba(colors.get(mesh_name))
            if rgba:
                if geom.get("rgba") != rgba:
                    geom.set("rgba", rgba)
                    changed += 1
            elif _is_black_rgba(geom.get("rgba")):
                geom.set("rgba", _DEFAULT_VISUAL_RGBA)
                changed += 1
    return changed


def _postprocess_root(root, *, pkg_dir, robot_name, loop_closures=None,
                      colors=None, mjcf_mod=None):
    if mjcf_mod is None:
        from . import mjcf_export as mjcf_mod

    cfg = _load_loop_closures(pkg_dir, loop_closures)
    closure_report = _add_closed_loop_equalities(
        root, cfg, pkg_dir, robot_name, mjcf_mod)
    actuator_report = _filter_actuators(root, cfg)
    color_count = _fix_visual_colors(root, colors)
    return {
        "closure": closure_report,
        "actuator": actuator_report,
        "visual_colors_fixed": color_count,
    }


def _append_readme(text, report):
    marker = "## CAD closed-loop export"
    if marker in text:
        return text
    c = report["closure"]
    a = report["actuator"]
    lines = [
        "",
        marker,
        "",
        f"- MuJoCo loop closures restored: {c['closures']} "
        f"({c['connects']} equality/connect constraints).",
        f"- Actuators kept: {a['after']} of {a['before']} tree-joint actuators; "
        "closed-loop dependent joints are passive.",
        f"- Visual geoms recoloured/fixed: {report['visual_colors_fixed']}.",
    ]
    if c["skipped"]:
        lines.append("- Closure warnings: " + "; ".join(c["skipped"]))
    return text.rstrip() + "\n" + "\n".join(lines) + "\n"


def _postprocess_path(out_root, *, pkg_dir, robot_name, loop_closures=None,
                      colors=None, mjcf_mod=None):
    mjcf_dir = os.path.join(out_root, "mjcf")
    xmls = sorted(
        os.path.join(mjcf_dir, f) for f in os.listdir(mjcf_dir)
        if f.lower().endswith(".xml"))
    if not xmls:
        return None
    path = xmls[0]
    tree = ET.parse(path)
    report = _postprocess_root(
        tree.getroot(), pkg_dir=pkg_dir, robot_name=robot_name,
        loop_closures=loop_closures, colors=colors, mjcf_mod=mjcf_mod)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="unicode", xml_declaration=False)

    readme = os.path.join(out_root, "README.md")
    if os.path.isfile(readme):
        try:
            with open(readme, encoding="utf-8") as f:
                text = f.read()
            with open(readme, "w", encoding="utf-8") as f:
                f.write(_append_readme(text, report))
        except OSError:
            pass
    return report


def _postprocess_files(files, *, pkg_dir, robot_name, loop_closures=None,
                       colors=None, mjcf_mod=None):
    out = []
    report = None
    readme_index = None
    for idx, (arc, data) in enumerate(files):
        if arc.lower().endswith(".xml") and "/mjcf/" in arc.replace("\\", "/"):
            root = ET.fromstring(data)
            report = _postprocess_root(
                root, pkg_dir=pkg_dir, robot_name=robot_name,
                loop_closures=loop_closures, colors=colors, mjcf_mod=mjcf_mod)
            ET.indent(root, space="  ")
            data = ET.tostring(root, encoding="utf-8")
        if arc.lower().endswith("/readme.md"):
            readme_index = idx
        out.append((arc, data))

    if report is not None and readme_index is not None:
        arc, data = out[readme_index]
        try:
            text = data.decode("utf-8")
            out[readme_index] = (arc, _append_readme(text, report).encode("utf-8"))
        except UnicodeDecodeError:
            pass
    return out


def install():
    """Patch the public MJCF export functions once for CLI and web callers."""
    from . import mjcf_export as mod

    if getattr(mod, "_cad_closed_loop_export_installed", False):
        return

    original_write = mod.write_mjcf_package
    original_build = mod.build_mjcf_package

    @wraps(original_write)
    def write_mjcf_package(*args, **kwargs):
        pkg_dir = args[0] if args else kwargs["pkg_dir"]
        robot_name = args[1] if len(args) > 1 else kwargs["robot_name"]
        out_root = original_write(*args, **kwargs)
        _postprocess_path(
            out_root, pkg_dir=pkg_dir, robot_name=robot_name,
            loop_closures=kwargs.get("loop_closures"),
            colors=kwargs.get("colors"), mjcf_mod=mod)
        return out_root

    @wraps(original_build)
    def build_mjcf_package(*args, **kwargs):
        pkg_dir = args[0] if args else kwargs["pkg_dir"]
        robot_name = args[1] if len(args) > 1 else kwargs["robot_name"]
        pkg, files = original_build(*args, **kwargs)
        files = _postprocess_files(
            files, pkg_dir=pkg_dir, robot_name=robot_name,
            loop_closures=kwargs.get("loop_closures"),
            colors=kwargs.get("colors"), mjcf_mod=mod)
        return pkg, files

    mod.write_mjcf_package = write_mjcf_package
    mod.build_mjcf_package = build_mjcf_package
    mod._cad_closed_loop_export_installed = True
