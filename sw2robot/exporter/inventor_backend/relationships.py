"""Map Inventor assembly relationships into sw2robot graph edges."""
from __future__ import annotations

import math
from collections import defaultdict

from ..state import LimitJoint, MateEdge
from .com import (
    CM_TO_M,
    geometry_direction,
    geometry_point,
    iter_collection,
    matrix_si,
    occ_name,
    point_m,
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


def _normal(v):
    if v is None:
        return None
    n = math.sqrt(sum(float(x) * float(x) for x in v))
    if n < 1e-12:
        return None
    return [float(x) / n for x in v]


def _occ_transform(occ, point, direction):
    """Transform an occurrence-local axis line into assembly coordinates."""
    if occ is None:
        return point, _normal(direction)
    m = matrix_si(safe_prop(occ, "Transformation"))
    r = [m[0:3], m[4:7], m[8:11]]
    t = [m[3], m[7], m[11]]
    p2 = None
    if point is not None:
        p2 = [
            sum(r[i][j] * float(point[j]) for j in range(3)) + t[i]
            for i in range(3)
        ]
    d2 = None
    if direction is not None:
        d2 = _normal([
            sum(r[i][j] * float(direction[j]) for j in range(3))
            for i in range(3)
        ])
    return p2, d2


def _is_proxy_geometry(obj):
    """Best-effort test for Inventor assembly-context geometry proxies."""
    if obj is None:
        return False
    if safe_prop(obj, "ContainingOccurrence") is not None:
        return True
    # EdgeProxy / FaceProxy / WorkAxisProxy expose NativeObject; their native
    # counterparts generally do not.  This is intentionally best-effort because
    # pywin32 does not preserve a convenient Python type hierarchy here.
    return safe_prop(obj, "NativeObject") is not None


def _intrinsic_axis_geometry(obj):
    """Does geometry itself define a line, rather than merely a plane normal?"""
    seen = set()
    for _ in range(4):
        if obj is None or id(obj) in seen:
            break
        seen.add(id(obj))
        if safe_prop(obj, "Center") is not None:
            return True
        if safe_prop(obj, "AxisVector") is not None:
            return True
        if safe_prop(obj, "Direction") is not None:
            return True
        nxt = safe_prop(obj, "Geometry")
        if nxt is None or nxt is obj:
            break
        obj = nxt
    return False


def _axis_variants(intent, occ):
    """Candidate assembly-space interpretations of one GeometryIntent.

    Existing Inventor joints can return either an assembly-context proxy or a
    definition-context/native object through COM, depending on how the joint was
    authored.  GeometryIntent.Point can also disagree with Geometry when one is
    definition-context and the other is a proxy.  Return several plausible axis
    lines and let OriginOne/OriginTwo consistency choose the physical one.
    """
    if intent is None:
        return []
    geom = safe_prop(intent, "Geometry")
    intent_point = point_m(safe_prop(intent, "Point"))
    geom_point = geometry_point(geom)
    direction = _normal(geometry_direction(geom))
    proxy = _is_proxy_geometry(geom)
    intrinsic = _intrinsic_axis_geometry(geom)

    variants = []

    def add(label, point, axis, bias):
        axis = _normal(axis)
        if point is None or axis is None:
            return
        key = tuple(round(float(x), 10) for x in point + axis)
        if any(v[4] == key for v in variants):
            return
        variants.append((label, point, axis, float(bias), key))

    # Autodesk proxies report coordinates in assembly context.  For circular,
    # cylindrical and linear geometry, the geometry point is guaranteed to lie
    # on the physical joint line, so prefer it over a possibly local Intent.Point.
    if geom_point is not None and direction is not None:
        add("proxy-geometry" if proxy else "geometry-as-returned",
            geom_point, direction, 0.0 if proxy and intrinsic else 0.002)
    if intent_point is not None and direction is not None:
        add("intent-as-returned", intent_point, direction,
            0.0005 if proxy else 0.003)

    # Native/definition-context geometry needs the occurrence transform to reach
    # the top-level assembly frame.  Also include this interpretation for proxy
    # data as a low-priority fallback because some Inventor COM combinations
    # expose a local GeometryIntent.Point next to an assembly-space Geometry.
    if geom_point is not None and direction is not None:
        p, d = _occ_transform(occ, geom_point, direction)
        add("geometry+occ-transform", p, d, 0.0 if not proxy else 0.01)
    if intent_point is not None and direction is not None:
        p, d = _occ_transform(occ, intent_point, direction)
        add("intent+occ-transform", p, d, 0.0005 if not proxy else 0.01)
        # Mixed-context fallback: Point may be local while Geometry direction is
        # already proxy/assembly-space.  This was observed as axes floating far
        # away while their orientation still looked correct.
        p_only, _ = _occ_transform(occ, intent_point, None)
        add("intent-point+occ / proxy-dir", p_only, direction,
            0.001 if proxy else 0.004)

    # For axis-bearing proxy geometry, reject a raw Intent.Point that is clearly
    # not on the proxy's physical line by making the proxy candidate dominant.
    if proxy and intrinsic and geom_point is not None and direction is not None:
        for i, rec in enumerate(variants):
            label, p, d, bias, key = rec
            if label != "intent-as-returned":
                continue
            delta = [p[j] - geom_point[j] for j in range(3)]
            cross = [
                delta[1] * direction[2] - delta[2] * direction[1],
                delta[2] * direction[0] - delta[0] * direction[2],
                delta[0] * direction[1] - delta[1] * direction[0],
            ]
            off_axis = math.sqrt(sum(x * x for x in cross))
            if off_axis > 1e-4:
                variants[i] = (label, p, d, bias + min(off_axis, 1.0), key)

    return variants


def _line_pair_score(v1, v2):
    """Small score when two candidate origin lines describe the same joint."""
    _, p1, d1, b1, _ = v1
    _, p2, d2, b2, _ = v2
    dot = max(-1.0, min(1.0, sum(d1[i] * d2[i] for i in range(3))))
    align_penalty = (1.0 - abs(dot)) * 0.25
    delta = [p2[i] - p1[i] for i in range(3)]
    cross = [
        delta[1] * d1[2] - delta[2] * d1[1],
        delta[2] * d1[0] - delta[0] * d1[2],
        delta[0] * d1[1] - delta[1] * d1[0],
    ]
    line_distance = math.sqrt(sum(x * x for x in cross))
    return line_distance + align_penalty + b1 + b2


def _native_joint_axis(definition, occ1=None, occ2=None):
    """Return authored Assembly Joint axis in TOP-LEVEL assembly coordinates.

    OriginOne and OriginTwo describe the same physical axis.  Inventor COM can
    expose their GeometryIntent data either through an assembly-context proxy or
    through native/local geometry.  Evaluate both interpretations and choose the
    pair whose axis lines coincide best.  This prevents a local point from being
    mistaken for a world point (the classic 'red joint axis floating in space'
    failure).
    """
    one = _axis_variants(safe_prop(definition, "OriginOne"), occ1)
    two = _axis_variants(safe_prop(definition, "OriginTwo"), occ2)

    chosen1 = chosen2 = None
    if one and two:
        chosen1, chosen2 = min(
            ((a, b) for a in one for b in two),
            key=lambda pair: _line_pair_score(pair[0], pair[1]),
        )
    elif one:
        chosen1 = min(one, key=lambda x: x[3])
    elif two:
        chosen2 = min(two, key=lambda x: x[3])

    chosen = chosen1 or chosen2
    if chosen is None:
        return None, None, "unavailable"

    label, point, axis, _bias, _keyv = chosen
    if bool(safe_prop(definition, "FlipOriginDirection", False)) and chosen1 is not None:
        axis = [-x for x in axis]
    source = label
    if chosen1 is not None and chosen2 is not None:
        source = f"OriginOne={chosen1[0]}, OriginTwo={chosen2[0]}"
    return point, _normal(axis), source


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
        point, axis, axis_source = _native_joint_axis(d, o1, o2)
        movable = (not locked and kind in (INV_ROTATIONAL, INV_SLIDE))

        diagnostics.append(
            f"joint {jname!r}: {label} {a} <-> {b}; "
            f"locked={locked}; p={_fmt_vec(point)} axis={_fmt_vec(axis)}; "
            f"coords={axis_source}"
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
    """Return (point, direction) in TOP-LEVEL ASSEMBLY coordinates."""
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
    """Read legacy assembly constraints not already represented by a Joint."""
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
