"""Pure-Python tests for the Inventor backend (no Inventor installation needed)."""
from __future__ import annotations

import math

import pytest

from sw2robot.exporter.inertia import MM_TO_M, mesh_scale_for_path
from sw2robot.exporter.inventor_backend.com import (
    KG_CM2_TO_KG_M2,
    matrix_si,
    point_m,
    xyz_inertia,
)
from sw2robot.exporter.inventor_backend.relationships import (
    INV_ROTATIONAL,
    INV_SLIDE,
    joint_limits,
)


class _Point:
    def __init__(self, x, y, z):
        self.X, self.Y, self.Z = x, y, z


class _Matrix:
    def __init__(self):
        self._v = [
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 1.0, 0.0, 20.0],
            [0.0, 0.0, 1.0, 30.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def Cell(self, r, c):
        return self._v[r - 1][c - 1]


class _MassPropsFallback:
    def XYZMomentsOfInertia(self, *args):
        # Autodesk API order: Ixx, Iyy, Izz, Ixy, Iyz, Ixz (kg*cm^2).
        return (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


class _Rotational:
    HasAngularPositionLimits = True
    AngularPositionStartLimit = -0.25
    AngularPositionEndLimit = 0.75


class _Slider:
    HasLinearPositionStartLimit = True
    HasLinearPositionEndLimit = True
    LinearPositionStartLimit = -12.5  # cm in the Inventor database
    LinearPositionEndLimit = 35.0


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
