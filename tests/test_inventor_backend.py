"""Pure-Python tests for the Inventor backend (no Inventor installation needed)."""
from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from sw2robot.exporter.inertia import MM_TO_M, mesh_scale_for_path
from sw2robot.exporter.inventor_backend.com import (
    KG_CM2_TO_KG_M2,
    matrix_si,
    point_m,
    xyz_inertia,
)
from sw2robot.exporter.inventor_backend.extract import (
    _assign_unique_link_names,
    _robot_safe_name,
)
from sw2robot.exporter.inventor_backend.relationships import (
    INV_PLANAR,
    INV_ROTATIONAL,
    INV_SLIDE,
    _constraint_axis,
    classic_constraints,
    joint_limits,
    native_joints,
)


class _Point:
    def __init__(self, x, y, z):
        self.X, self.Y, self.Z = x, y, z


class _Vector:
    def __init__(self, x, y, z):
        self.X, self.Y, self.Z = x, y, z


class _Geometry:
    def __init__(self, center, normal):
        self.Center = _Point(*center)
        self.Normal = _Vector(*normal)


class _ProxyGeometry(_Geometry):
    def __init__(self, center, normal):
        super().__init__(center, normal)
        self.NativeObject = object()
        self.ContainingOccurrence = object()


class _Intent:
    def __init__(self, point, direction, geometry=None):
        self.Point = _Point(*point)
        self.Geometry = geometry or _Geometry(point, direction)


class _Collection:
    def __init__(self, *items):
        self._items = list(items)
        self.Count = len(self._items)

    def Item(self, i):
        return self._items[i - 1]


class _Matrix:
    def __init__(self, tx=10.0, ty=20.0, tz=30.0):
        self._v = [
            [1.0, 0.0, 0.0, tx],
            [0.0, 1.0, 0.0, ty],
            [0.0, 0.0, 1.0, tz],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def Cell(self, r, c):
        return self._v[r - 1][c - 1]


class _MassPropsFallback:
    def XYZMomentsOfInertia(self, *args):
        return (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


class _Rotational:
    HasAngularPositionLimits = True
    AngularPositionStartLimit = -0.25
    AngularPositionEndLimit = 0.75


class _Slider:
    HasLinearPositionStartLimit = True
    HasLinearPositionEndLimit = True
    LinearPositionStartLimit = -12.5
    LinearPositionEndLimit = 35.0


def _joint(name, kind, a, b, point=(10.0, 20.0, 30.0), axis=(0.0, 1.0, 0.0)):
    d = SimpleNamespace(
        JointType=kind,
        OriginOne=_Intent(point, axis),
        OriginTwo=_Intent(point, axis),
        FlipOriginDirection=False,
        HasAngularPositionLimits=False,
        HasLinearPositionStartLimit=False,
        HasLinearPositionEndLimit=False,
    )
    oa = SimpleNamespace(Name=a)
    ob = SimpleNamespace(Name=b)
    return SimpleNamespace(
        Name=name,
        Suppressed=False,
        Locked=False,
        Definition=d,
        AffectedOccurrenceOne=oa,
        AffectedOccurrenceTwo=ob,
        OccurrenceOne=oa,
        OccurrenceTwo=ob,
    )


def test_point_converts_inventor_cm_to_m():
    assert point_m(_Point(10.0, -25.0, 2.5)) == pytest.approx([0.1, -0.25, 0.025])


def test_matrix_translation_converts_cm_to_m_only():
    m = matrix_si(_Matrix())
    assert m[:4] == pytest.approx([1.0, 0.0, 0.0, 0.1])
    assert m[4:8] == pytest.approx([0.0, 1.0, 0.0, 0.2])
    assert m[8:12] == pytest.approx([0.0, 0.0, 1.0, 0.3])
    assert m[12:] == pytest.approx([0.0, 0.0, 0.0, 1.0])


def test_xyz_inertia_reorders_and_converts_kg_cm2_to_kg_m2():
    got = xyz_inertia(_MassPropsFallback())
    s = KG_CM2_TO_KG_M2
    assert got == pytest.approx([1 * s, 4 * s, 6 * s, 2 * s, 5 * s, 3 * s])


def test_inventor_stl_is_already_in_metres():
    assert mesh_scale_for_path("meshes/link.stl") == 1.0
    assert mesh_scale_for_path("meshes/composed.glb") == 1.0
    assert mesh_scale_for_path("meshes/solidworks.3dxml") == MM_TO_M


def test_chinese_robot_name_does_not_collapse_to_plain_c_prefix():
    got = _robot_safe_name("整体装配体")
    assert got.startswith("robot_")
    assert len(got) > len("robot_")
    assert got == _robot_safe_name("整体装配体")


def test_chinese_robot_name_with_digits_does_not_become_c_2():
    got = _robot_safe_name("整体装配体2")
    assert got.startswith("robot_")
    assert got != "c_2"
    assert got == _robot_safe_name("整体装配体2")


def test_mixed_ascii_unicode_robot_name_keeps_ascii_part():
    assert _robot_safe_name("robot_整机2") == "robot_2"


def test_unicode_occurrences_that_collapse_to_same_ascii_get_unique_links():
    comps = [
        SimpleNamespace(name="大臂大孔:1", link_name=""),
        SimpleNamespace(name="假铝柱:1", link_name=""),
        SimpleNamespace(name="测试模型，壳体:1", link_name=""),
        SimpleNamespace(name="DM-J4310:1", link_name=""),
    ]
    _assign_unique_link_names(comps)
    names = [c.link_name for c in comps]
    assert len(names) == len(set(names))
    assert all(n.startswith("c_1_") for n in names[:3])
    assert names[3] == "DM_J4310_1"


def test_exact_duplicate_occurrence_names_are_still_forced_unique():
    comps = [SimpleNamespace(name="零件:1", link_name=""),
             SimpleNamespace(name="零件:1", link_name="")]
    _assign_unique_link_names(comps)
    assert comps[0].link_name != comps[1].link_name


def test_constraint_axis_prefers_documented_assembly_space_geometry():
    constraint = SimpleNamespace(
        GeometryOne=_Geometry((10.0, 20.0, 30.0), (0.0, 1.0, 0.0)),
        GeometryTwo=None,
        EntityOne=_Geometry((900.0, 800.0, 700.0), (1.0, 0.0, 0.0)),
        EntityTwo=None,
    )
    point, axis = _constraint_axis(constraint)
    assert point == pytest.approx([0.1, 0.2, 0.3])
    assert axis == pytest.approx([0.0, 1.0, 0.0])


def test_non_insert_classic_constraint_is_not_force_fixed():
    a = SimpleNamespace(Name="body:1")
    b = SimpleNamespace(Name="bracket:1")
    constraint = SimpleNamespace(
        Suppressed=False,
        AffectedOccurrenceOne=a,
        AffectedOccurrenceTwo=b,
        OccurrenceOne=a,
        OccurrenceTwo=b,
    )
    definition = SimpleNamespace(Constraints=_Collection(constraint))
    edges, limits, ground = classic_constraints(
        definition, {"body:1", "bracket:1"}, set())
    edge = next(iter(edges.values()))
    assert edge.force_fixed is False
    assert edge.types == ["INVENTOR_CONSTRAINT"]
    assert limits == []
    assert ground == set()


def test_classic_insert_cannot_add_dof_in_native_joint_mode():
    a = SimpleNamespace(Name="body:1")
    b = SimpleNamespace(Name="arm:1")
    insert = SimpleNamespace(
        Suppressed=False,
        AffectedOccurrenceOne=a,
        AffectedOccurrenceTwo=b,
        OccurrenceOne=a,
        OccurrenceTwo=b,
        AxesOpposed=False,
        GeometryOne=_Geometry((10.0, 20.0, 30.0), (0.0, 1.0, 0.0)),
        GeometryTwo=None,
        EntityOne=None,
        EntityTwo=None,
    )
    definition = SimpleNamespace(Constraints=_Collection(insert))
    edges, limits, _ = classic_constraints(
        definition, {"body:1", "arm:1"}, set(), allow_movable=False)
    assert limits == []
    assert next(iter(edges.values())).types == ["INVENTOR_INSERT_FIXED"]


def test_authored_rotational_wins_over_planar_on_same_pair():
    a, b = "DM-J4310:1", "大臂大孔:1"
    rotational = _joint("ACT_HIP_FL", INV_ROTATIONAL, a, b)
    planar = _joint("old planar", INV_PLANAR, a, b)
    definition = SimpleNamespace(Joints=_Collection(rotational, planar))
    edges, limits, _ground, _covered, warnings, diagnostics = native_joints(
        definition, {a, b})
    assert len(edges) == 1
    edge = next(iter(edges.values()))
    assert edge.force_fixed is False
    assert "INVENTOR_ROTATIONAL" in edge.types
    assert "INVENTOR_PLANAR" in edge.types
    assert edge.types.count("CONCENTRIC") == 2
    assert len(limits) == 1
    assert limits[0].type == "revolute"
    assert limits[0].axis_point == pytest.approx([0.1, 0.2, 0.3])
    assert limits[0].axis_dir == pytest.approx([0.0, 1.0, 0.0])
    assert any("ACT_HIP_FL" in line for line in diagnostics)
    assert any("INVENTOR_PLANAR" in msg for msg in warnings)


def test_native_joint_local_geometry_is_transformed_into_assembly_space():
    a, b = "motor:1", "arm:1"
    # Both native origins describe the same physical X=110 cm joint axis, but
    # each point is expressed in its own occurrence-local coordinates.
    o1 = SimpleNamespace(Name=a, Transformation=_Matrix(100.0, 0.0, 0.0))
    o2 = SimpleNamespace(Name=b, Transformation=_Matrix(110.0, 0.0, 0.0))
    d = SimpleNamespace(
        JointType=INV_ROTATIONAL,
        OriginOne=_Intent((10.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        OriginTwo=_Intent((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        FlipOriginDirection=False,
        HasAngularPositionLimits=False,
    )
    j = SimpleNamespace(
        Name="ACT_TEST", Suppressed=False, Locked=False, Definition=d,
        AffectedOccurrenceOne=o1, AffectedOccurrenceTwo=o2,
        OccurrenceOne=o1, OccurrenceTwo=o2,
    )
    definition = SimpleNamespace(Joints=_Collection(j))
    _edges, limits, *_ = native_joints(definition, {a, b})
    assert limits[0].axis_point == pytest.approx([1.1, 0.0, 0.0])
    assert limits[0].axis_dir == pytest.approx([0.0, 0.0, 1.0])


def test_native_joint_proxy_geometry_is_not_transformed_twice():
    a, b = "motor:1", "arm:1"
    # Proxy geometry is already in assembly space at X=110 cm.  Occurrence
    # transforms are intentionally non-identity; applying them again would move
    # the red axis far away from the actual model joint.
    o1 = SimpleNamespace(Name=a, Transformation=_Matrix(100.0, 0.0, 0.0))
    o2 = SimpleNamespace(Name=b, Transformation=_Matrix(110.0, 0.0, 0.0))
    world_geom1 = _ProxyGeometry((110.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    world_geom2 = _ProxyGeometry((110.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    d = SimpleNamespace(
        JointType=INV_ROTATIONAL,
        OriginOne=_Intent((110.0, 0.0, 0.0), (0.0, 0.0, 1.0), world_geom1),
        OriginTwo=_Intent((110.0, 0.0, 0.0), (0.0, 0.0, 1.0), world_geom2),
        FlipOriginDirection=False,
        HasAngularPositionLimits=False,
    )
    j = SimpleNamespace(
        Name="ACT_PROXY", Suppressed=False, Locked=False, Definition=d,
        AffectedOccurrenceOne=o1, AffectedOccurrenceTwo=o2,
        OccurrenceOne=o1, OccurrenceTwo=o2,
    )
    definition = SimpleNamespace(Joints=_Collection(j))
    _edges, limits, *_ = native_joints(definition, {a, b})
    assert limits[0].axis_point == pytest.approx([1.1, 0.0, 0.0])
    assert limits[0].axis_dir == pytest.approx([0.0, 0.0, 1.0])


def test_rotational_joint_limits_stay_in_radians():
    assert joint_limits(_Rotational(), INV_ROTATIONAL) == pytest.approx((-0.25, 0.75))


def test_slider_joint_limits_convert_cm_to_m():
    assert joint_limits(_Slider(), INV_SLIDE) == pytest.approx((-0.125, 0.35))


def test_unlimited_rotation_gets_safe_editor_range():
    class D:
        HasAngularPositionLimits = False

    lo, hi = joint_limits(D(), INV_ROTATIONAL)
    assert lo == pytest.approx(-math.pi)
    assert hi == pytest.approx(math.pi)
