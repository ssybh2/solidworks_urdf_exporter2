"""High-level Inventor IAM/IPT -> sw2robot package extraction."""
from __future__ import annotations

import argparse
import os

from ..model import safe_name
from ..state import ComponentState, GraphState
from .com import (Inventor, doc_path, export_stl, iter_collection, mass_props,
                  matrix_si, mesh_name, occ_doc, occ_name, safe_prop, ucs_states)
from .relationships import classic_constraints, native_joints

GRAPH_FILE = "graph.json"


def _package_paths(cad_path, out_dir, robot_name):
    robot_name = safe_name(robot_name or os.path.splitext(os.path.basename(cad_path))[0])
    out_dir = os.path.abspath(out_dir or os.path.join(os.getcwd(), "output"))
    return robot_name, os.path.join(out_dir, robot_name)


def _emit(progress, msg):
    line = f"[inventor] {msg}"
    print(line)
    if progress is not None:
        progress(msg)


def _component_state(occ, meshes_dir, app, cache):
    name = occ_name(occ)
    doc, definition = occ_doc(occ), safe_prop(occ, "Definition")
    path = doc_path(doc)
    mass, com, inertia, overridden = mass_props(definition)
    mesh_rel = None
    if path and doc is not None:
        key = os.path.normcase(path)
        mesh_abs = cache.get(key) or os.path.join(meshes_dir, mesh_name(path))
        if key not in cache and not os.path.exists(mesh_abs):
            export_stl(app, doc, mesh_abs)
        cache[key] = mesh_abs
        mesh_rel = f"meshes/{os.path.basename(mesh_abs)}"
    return ComponentState(
        name=name, link_name=safe_name(name), part_path=path,
        is_subassembly=bool(path and path.lower().endswith(".iam")),
        world=matrix_si(safe_prop(occ, "Transformation")),
        fixed=bool(safe_prop(occ, "Grounded", False)), mesh_file=mesh_rel,
        sw_mass=mass, sw_com=com, sw_inertia=inertia,
        sw_mass_overridden=overridden)


def extract_inventor(cad_path, out_dir=None, robot_name=None,
                     visible=False, attach=False, progress=None):
    """Extract an Inventor IAM/IPT into the CAD-independent sw2robot package.

    ``progress`` is an optional callable receiving human-readable phase strings;
    the browser server uses it for live progress and cooperative cancellation.
    """
    cad_path = os.path.abspath(cad_path)
    ext = os.path.splitext(cad_path)[1].lower()
    if ext not in (".iam", ".ipt"):
        raise ValueError("Inventor backend accepts .iam assembly or .ipt part files")
    robot_name, pkg_dir = _package_paths(cad_path, out_dir, robot_name)
    meshes_dir = os.path.join(pkg_dir, "meshes")
    os.makedirs(meshes_dir, exist_ok=True)

    _emit(progress, f"opening {os.path.basename(cad_path)} ...")
    with Inventor(visible=visible, attach=attach) as inv:
        doc = inv.open(cad_path)
        try:
            definition = safe_prop(doc, "ComponentDefinition")
            if definition is None:
                raise ValueError("Inventor document has no ComponentDefinition")
            if ext == ".ipt":
                mass, com, inertia, overridden = mass_props(definition)
                mesh_abs = os.path.join(meshes_dir, mesh_name(cad_path))
                _emit(progress, "exporting mesh 1/1: " + os.path.basename(cad_path))
                export_stl(inv.app, doc, mesh_abs)
                graph = GraphState(
                    robot_name=robot_name, source_assembly=cad_path,
                    components=[ComponentState(
                        name=robot_name, link_name=robot_name, part_path=cad_path,
                        world=[1.,0.,0.,0., 0.,1.,0.,0., 0.,0.,1.,0., 0.,0.,0.,1.],
                        fixed=True, mesh_file=f"meshes/{os.path.basename(mesh_abs)}",
                        sw_mass=mass, sw_com=com, sw_inertia=inertia,
                        sw_mass_overridden=overridden)],
                    ground=[robot_name], coordinate_systems=ucs_states(definition))
            else:
                _emit(progress, "reading assembly occurrences and joints ...")
                occs = [o for o in iter_collection(safe_prop(definition, "Occurrences"))
                        if not bool(safe_prop(o, "Suppressed", False))]
                _emit(progress, f"components: {len(occs)}")
                cache, components, hidden, part_frames = {}, [], [], {}
                for i, occ in enumerate(occs, 1):
                    _emit(progress,
                          f"exporting mesh {i}/{len(occs)}: {occ_name(occ)}")
                    comp = _component_state(occ, meshes_dir, inv.app, cache)
                    components.append(comp)
                    if not bool(safe_prop(occ, "Visible", True)):
                        hidden.append(comp.name)
                    if comp.part_path and comp.part_path not in part_frames:
                        frames = ucs_states(safe_prop(occ, "Definition"))
                        if frames:
                            part_frames[comp.part_path] = frames

                names = {c.name for c in components}
                edges, limits, ground, covered, warnings = native_joints(definition, names)
                extra_edges, extra_limits, extra_ground = classic_constraints(
                    definition, names, covered)
                edges.update(extra_edges)
                limits.extend(extra_limits)
                ground.update(extra_ground)
                ground.update(c.name for c in components if c.fixed)
                if not ground and components:
                    ground.add(components[0].name)
                for warning in warnings:
                    _emit(progress, f"WARN: {warning}")
                _emit(progress, f"relationships: {len(edges)} edge(s), "
                                f"{len(limits)} movable")
                graph = GraphState(
                    robot_name=robot_name, source_assembly=cad_path,
                    components=components, edges=list(edges.values()),
                    ground=sorted(ground), coordinate_systems=ucs_states(definition),
                    part_coordinate_systems=part_frames, hidden=hidden,
                    limit_joints=limits)

            graph_path = os.path.join(pkg_dir, GRAPH_FILE)
            graph.save(graph_path)
            _emit(progress, f"graph: {graph_path}")
        finally:
            inv.close(doc)
    return pkg_dir


def main():
    ap = argparse.ArgumentParser(
        description="Autodesk Inventor IAM/IPT -> sw2robot URDF/MJCF")
    ap.add_argument("cad", help="path to .iam assembly or .ipt part")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("-n", "--name", default=None)
    ap.add_argument("--visible", action="store_true")
    ap.add_argument("--attach", action="store_true",
                    help="reuse an already-running Inventor instance")
    ap.add_argument("--extract-only", action="store_true")
    ap.add_argument("--mujoco", action="store_true")
    ap.add_argument("--mujoco-fixed-base", action="store_true")
    args = ap.parse_args()
    pkg = extract_inventor(args.cad, args.out, args.name, args.visible, args.attach)
    if not args.extract_only:
        from ..export import build
        build(pkg, mujoco=args.mujoco,
              mujoco_fixed_base=args.mujoco_fixed_base)
        print(f"[inventor] package ready: {pkg}")
