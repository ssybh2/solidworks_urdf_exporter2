"""Map Inventor assembly relationships into sw2robot graph edges."""
from __future__ import annotations

import math
from collections import defaultdict

from ..state import LimitJoint, MateEdge
from .com import CM_TO_M, geometry_direction, geometry_point, intent_axis, iter_collection, occ_name, safe_prop

INV_RIGID = 102401
INV_ROTATIONAL = 102402
INV_SLIDE = 102403
INV_CYLINDRICAL = 102404
INV_PLANAR = 102405
INV_BALL = 102406


def _numeric(v, default=None):
    try:
        return float(v)
    except Exception:
        try:
            return float(v.Value)
        except Exception:
            return default


def joint_limits(definition, kind):
    if kind == INV_ROTATIONAL:
        if bool(safe_prop(definition, "HasAngularPositionLimits", False)):
            return (_numeric(safe_prop(definition, "AngularPositionStartLimit"), -math.pi),
                    _numeric(safe_prop(definition, "AngularPositionEndLimit"), math.pi))
        return -math.pi, math.pi
    if kind == INV_SLIDE:
        has_lo = bool(safe_prop(definition, "HasLinearPositionStartLimit", False))
        has_hi = bool(safe_prop(definition, "HasLinearPositionEndLimit", False))
        lo = _numeric(safe_prop(definition, "LinearPositionStartLimit"), -100.) if has_lo else -100.
        hi = _numeric(safe_prop(definition, "LinearPositionEndLimit"), 100.) if has_hi else 100.
        return lo * CM_TO_M, hi * CM_TO_M
    return None, None


def _key(a, b):
    return frozenset((a, b))


def native_joints(definition, names):
    labels = {INV_RIGID: "INVENTOR_RIGID", INV_ROTATIONAL: "INVENTOR_ROTATIONAL",
              INV_SLIDE: "INVENTOR_SLIDE", INV_CYLINDRICAL: "INVENTOR_CYLINDRICAL",
              INV_PLANAR: "INVENTOR_PLANAR", INV_BALL: "INVENTOR_BALL"}
    edges, limits, ground, covered, warnings = {}, [], set(), set(), []
    for joint in iter_collection(safe_prop(definition, "Joints")):
        if bool(safe_prop(joint, "Suppressed", False)):
            continue
        d = safe_prop(joint, "Definition")
        if d is None:
            continue
        # AffectedOccurrence* is the assembly-context occurrence Autodesk says
        # should be used for adaptive/nested relationships.  Fall back to the
        # owning occurrence for older/simple documents.
        o1 = safe_prop(joint, "AffectedOccurrenceOne") or safe_prop(joint, "OccurrenceOne")
        o2 = safe_prop(joint, "AffectedOccurrenceTwo") or safe_prop(joint, "OccurrenceTwo")
        a, b = occ_name(o1), occ_name(o2)
        if a not in names or b not in names:
            grounded = b if a not in names else a
            if grounded in names:
                ground.add(grounded)
            continue
        key = _key(a, b)
        covered.add(key)
        kind = int(safe_prop(d, "JointType", 0) or 0)
        point, axis = intent_axis(safe_prop(d, "OriginOne"),
                                  flip=bool(safe_prop(d, "FlipOriginDirection", False)))
        if point is None:
            point, axis2 = intent_axis(safe_prop(d, "OriginTwo"))
            axis = axis or axis2
        locked = bool(safe_prop(joint, "Locked", False))
        edges[key] = MateEdge(a=a, b=b,
            types=[labels.get(kind, f"INVENTOR_JOINT_{kind}")],
            axis_point=point, axis_dir=axis,
            force_fixed=(kind == INV_RIGID or locked))
        if not locked and kind in (INV_ROTATIONAL, INV_SLIDE):
            if axis is None:
                axis = [0., 0., 1.]
                warnings.append(f"{a}<->{b}: joint axis unavailable; using +Z")
            point = point or [0., 0., 0.]
            lo, hi = joint_limits(d, kind)
            limits.append(LimitJoint(a=a, b=b,
                type="revolute" if kind == INV_ROTATIONAL else "prismatic",
                axis_point=point, axis_dir=axis, lower=lo, upper=hi))
        elif not locked and kind in (INV_CYLINDRICAL, INV_PLANAR, INV_BALL):
            warnings.append(
                f"{a}<->{b}: {labels[kind]} is multi-DOF; first pass keeps it fixed")
    return edges, limits, ground, covered, warnings


def _constraint_occurrence(constraint, which):
    """Return an assembly-context occurrence for a classic constraint."""
    return (safe_prop(constraint, f"AffectedOccurrence{which}")
            or safe_prop(constraint, f"Occurrence{which}"))


def _constraint_axis(constraint):
    """Return (point, direction) in TOP-LEVEL ASSEMBLY coordinates.

    Autodesk documents AssemblyConstraint.GeometryOne/GeometryTwo as geometry
    expressed in assembly space.  EntityOne/EntityTwo can be definition-context
    entities, so using those directly as world coordinates makes a URDF joint
    pivot around a remote/wrong point.  Always prefer Geometry* and keep Entity*
    only as a compatibility fallback.
    """
    for suffix in ("One", "Two"):
        geom = safe_prop(constraint, f"Geometry{suffix}")
        point, axis = geometry_point(geom), geometry_direction(geom)
        if point is not None and axis is not None:
            return point, axis
    for suffix in ("One", "Two"):
        entity = safe_prop(constraint, f"Entity{suffix}")
        point, axis = geometry_point(entity), geometry_direction(entity)
        if point is not None and axis is not None:
            return point, axis
    return None, None


def classic_constraints(definition, names, covered):
    """Fallback: classic Insert constraints -> revolute; other pairs -> fixed.

    InsertConstraint is treated as an explicit 1-DOF joint.  Other classic
    constraints describe assembly relationships but are deliberately NOT marked
    ``force_fixed``: a mate/flush/alignment constraint can be part of a loop
    around a real Insert joint, and making every such edge high-priority rigid
    lets the spanning tree bypass (and therefore drop) a real motor joint.
    """
    grouped, ground = defaultdict(list), set()
    for constraint in iter_collection(safe_prop(definition, "Constraints")):
        if bool(safe_prop(constraint, "Suppressed", False)):
            continue
        a = occ_name(_constraint_occurrence(constraint, "One"))
        b = occ_name(_constraint_occurrence(constraint, "Two"))
        if a not in names or b not in names:
            grounded = b if a not in names else a
            if grounded in names:
                ground.add(grounded)
            continue
        key = _key(a, b)
        if key not in covered:
            grouped[key].append((constraint, a, b))

    edges, limits = {}, []
    for key, records in grouped.items():
        _, a, b = records[0]
        # InsertConstraint uniquely exposes AxesOpposed in the classic API.
        insert = next((c for c, _, _ in records
                       if safe_prop(c, "AxesOpposed", None) is not None), None)
        if insert is None:
            edges[key] = MateEdge(a=a, b=b, types=["INVENTOR_CONSTRAINT"])
            continue

        # GeometryOne/Two are documented by Autodesk as ASSEMBLY-SPACE geometry
        # and are therefore the correct URDF world-axis source.
        point, axis = _constraint_axis(insert)
        point, axis = point or [0., 0., 0.], axis or [0., 0., 1.]
        edges[key] = MateEdge(a=a, b=b, types=["INVENTOR_INSERT"],
                              axis_point=point, axis_dir=axis)
        limits.append(LimitJoint(a=a, b=b, type="revolute",
                                 axis_point=point, axis_dir=axis,
                                 lower=-math.pi, upper=math.pi))
    return edges, limits, ground
