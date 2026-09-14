"""Map Inventor assembly relationships into sw2robot graph edges."""
from __future__ import annotations

import math
from collections import defaultdict

from ..state import LimitJoint, MateEdge
from .com import (
    CM_TO_M,
    geometry_direction,
    geometry_point,
    intent_axis,
    iter_collection,
    occ_name,
    safe_prop,
)

INV_RIGID = 102401
INV_ROTATIONAL = 102402
INV_SLIDE = 102403
INV_CYLINDRICAL = 102404
INV_PLANAR = 102405
INV_BALL = 102406

_LABELS = {
    INV_RIGID: "INVENTOR_RIGID",
    INV_ROTATIONAL: "INVENTOR_ROTATIONAL",
    INV_SLIDE: "INVENTOR_SLIDE",
    INV_CYLINDRICAL: "INVENTOR_CYLINDRICAL",
    INV_PLANAR: "INVENTOR_PLANAR",
    INV_BALL: "INVENTOR_BALL",
}


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
            return (
                _numeric(safe_prop(definition, "AngularPositionStartLimit"), -math.pi),
                _numeric(safe_prop(definition, "AngularPositionEndLimit"), math.pi),
            )
        return -math.pi, math.pi
    if kind == INV_SLIDE:
        has_lo = bool(safe_prop(definition, "HasLinearPositionStartLimit", False))
        has_hi = bool(safe_prop(definition, "HasLinearPositionEndLimit", False))
        lo = (
            _numeric(safe_prop(definition, "LinearPositionStartLimit"), -100.0)
            if has_lo else -100.0
        )
        hi = (
            _numeric(safe_prop(definition, "LinearPositionEndLimit"), 100.0)
            if has_hi else 100.0
        )
        return lo * CM_TO_M, hi * CM_TO_M
    return None, None


def _key(a, b):
    return frozenset((a, b))


def _fmt_vec(v):
    if v is None:
        return "?"
    return "(" + ", ".join(f"{float(x):.6g}" for x in v) + ")"


def _native_joint_axis(definition):
    """Return the authored Assembly Joint axis in assembly coordinates.

    Inventor Assembly Joint origins are GeometryIntent objects created in the
    AssemblyComponentDefinition context.  OriginOne is the authoritative joint
    origin/direction; OriginTwo is a fallback when a particular geometry intent
    does not expose a point or direction through COM.
    """
    flip = bool(safe_prop(definition, "FlipOriginDirection", False))
    p1, d1 = intent_axis(safe_prop(definition, "OriginOne"), flip=flip)
    p2, d2 = intent_axis(safe_prop(definition, "OriginTwo"), flip=False)
    point = p1 if p1 is not None else p2
    axis = d1 if d1 is not None else d2
    if axis is not None:
        n = math.sqrt(sum(float(x) * float(x) for x in axis))
        if n > 1e-12:
            axis = [float(x) / n for x in axis]
        else:
            axis = None
    return point, axis


def native_joints(definition, names):
    """Read explicit Inventor Assembly Joints.

    Rotational/Slide joints are authoritative motion declarations.  A pair may
    have more than one Inventor relationship (for example an old planar joint
    plus a newly-authored rotational joint); do not let the last COM item win.
    Instead aggregate the pair and let an explicit 1-DOF joint own its axis and
    movable state.
    """
    pair_records = {}
    limits, ground, covered, warnings, diagnostics = [], set(), set(), [], []

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
        label = _LABELS.get(kind, f"INVENTOR_JOINT_{kind}")
        jname = str(safe_prop(joint, "Name", "") or label)
        locked = bool(safe_prop(joint, "Locked", False))
        point, axis = _native_joint_axis(d)
        movable = (not locked and kind in (INV_ROTATIONAL, INV_SLIDE))

        diagnostics.append(
            f"joint {jname!r}: {label} {a} <-> {b}; "
            f"locked={locked}; p={_fmt_vec(point)} axis={_fmt_vec(axis)}"
        )

        rec = pair_records.setdefault(key, {
            "a": a,
            "b": b,
            "types": [],
            "axis_point": None,
            "axis_dir": None,
            "force_fixed": False,
            "movable": False,
        })
        rec["types"].append(label)

        if movable:
            # The shared tree builder prefers mate-richer edges.  Two synthetic
            # CONCENTRIC tags make a user-authored Inventor joint outrank plain
            # fixed/legacy constraints without changing the authoritative
            # LimitJoint type below (Slide still becomes prismatic).
            rec["types"].extend(["CONCENTRIC", "CONCENTRIC"])
            rec["movable"] = True
            rec["force_fixed"] = False
            if axis is None:
                axis = [0.0, 0.0, 1.0]
                warnings.append(
                    f"{jname}: {a}<->{b} joint axis unavailable; using +Z"
                )
            if point is None:
                point = [0.0, 0.0, 0.0]
                warnings.append(
                    f"{jname}: {a}<->{b} joint origin unavailable; using assembly origin"
                )
            rec["axis_point"] = point
            rec["axis_dir"] = axis
            lo, hi = joint_limits(d, kind)
            limits.append(LimitJoint(
                a=a,
                b=b,
                type="revolute" if kind == INV_ROTATIONAL else "prismatic",
                axis_point=point,
                axis_dir=axis,
                lower=lo,
                upper=hi,
            ))
        else:
            # Rigid/locked relationships are truly fixed.  Unsupported
            # multi-DOF joints are represented as fixed for now, but are NOT
            # force-fixed so they cannot outrank a real authored 1-DOF joint on
            # another path through a closed mechanism.
            if not rec["movable"] and (kind == INV_RIGID or locked):
                rec["force_fixed"] = True
            if rec["axis_point"] is None and point is not None:
                rec["axis_point"] = point
                rec["axis_dir"] = axis
            if not locked and kind in (INV_CYLINDRICAL, INV_PLANAR, INV_BALL):
                warnings.append(
                    f"{a}<->{b}: {label} is multi-DOF; current exporter keeps it fixed"
                )

    edges = {}
    for key, rec in pair_records.items():
        edges[key] = MateEdge(
            a=rec["a"],
            b=rec["b"],
            types=rec["types"],
            axis_point=rec["axis_point"],
            axis_dir=rec["axis_dir"],
            force_fixed=bool(rec["force_fixed"] and not rec["movable"]),
        )

    return edges, limits, ground, covered, warnings, diagnostics


def _constraint_occurrence(constraint, which):
    """Return an assembly-context occurrence for a classic constraint."""
    return (
        safe_prop(constraint, f"AffectedOccurrence{which}")
        or safe_prop(constraint, f"Occurrence{which}")
    )


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


def classic_constraints(definition, names, covered, allow_movable=True):
    """Read legacy assembly constraints not already represented by a Joint.

    When the assembly contains explicit Rotational/Slide Assembly Joints,
    ``allow_movable`` is False: old Insert constraints may still exist purely to
    position hardware, but they must not create extra URDF revolute joints.  They
    are retained as fixed graph connectivity only.
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
        insert = next(
            (c for c, _, _ in records if safe_prop(c, "AxesOpposed", None) is not None),
            None,
        )

        if insert is None or not allow_movable:
            edges[key] = MateEdge(
                a=a,
                b=b,
                types=["INVENTOR_CONSTRAINT" if insert is None else "INVENTOR_INSERT_FIXED"],
            )
            continue

        # GeometryOne/Two are documented by Autodesk as ASSEMBLY-SPACE geometry
        # and are therefore the correct URDF world-axis source.
        point, axis = _constraint_axis(insert)
        point, axis = point or [0.0, 0.0, 0.0], axis or [0.0, 0.0, 1.0]
        edges[key] = MateEdge(
            a=a,
            b=b,
            types=["INVENTOR_INSERT"],
            axis_point=point,
            axis_dir=axis,
        )
        limits.append(LimitJoint(
            a=a,
            b=b,
            type="revolute",
            axis_point=point,
            axis_dir=axis,
            lower=-math.pi,
            upper=math.pi,
        ))
    return edges, limits, ground
