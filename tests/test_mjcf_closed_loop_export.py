import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from sw2robot.exporter import exporter_fixes as F


class _FakeMjcf:
    @staticmethod
    def _world_frames(worldbody, home):
        del home
        frames = {}

        def walk(parent, R, p):
            for body in parent.findall("body"):
                pos = np.asarray(
                    [float(x) for x in body.get("pos", "0 0 0").split()],
                    dtype=float,
                )
                here = p + R @ pos
                frames[body.get("name")] = (R.copy(), here)
                walk(body, R, here)

        walk(worldbody, np.eye(3), np.zeros(3))
        return frames

    @staticmethod
    def _base_body(worldbody):
        bodies = worldbody.findall("body")
        return bodies[0] if bodies else None


def _write_working_urdf(tmp_path, fixed_endpoint=False):
    urdf = tmp_path / "urdf"
    urdf.mkdir()
    if fixed_endpoint:
        text = """<robot name="robot">
<link name="base_link"/><link name="a"/><link name="a_mount"/><link name="b"/>
<joint name="driver" type="revolute"><parent link="base_link"/><child link="a"/></joint>
<joint name="mount" type="fixed"><parent link="a"/><child link="a_mount"/></joint>
<joint name="passive" type="revolute"><parent link="base_link"/><child link="b"/></joint>
</robot>"""
    else:
        text = """<robot name="robot">
<link name="base_link"/><link name="a"/><link name="b"/>
<joint name="driver" type="revolute"><parent link="base_link"/><child link="a"/></joint>
<joint name="passive" type="revolute"><parent link="base_link"/><child link="b"/></joint>
</robot>"""
    (urdf / "robot.urdf").write_text(text, encoding="utf-8")


def _mjcf_root():
    return ET.fromstring("""<mujoco>
<worldbody>
  <body name="base_link">
    <geom type="sphere" size="0.01" density="1000" contype="0" conaffinity="0"/>
    <body name="a" pos="0.1 0 0">
      <joint name="driver" type="hinge" axis="1 0 0"/>
      <geom name="a_visual" type="sphere" size="0.01" density="1000"
            group="2" contype="0" conaffinity="0" rgba="0 0 0 1"/>
    </body>
    <body name="b" pos="-0.1 0 0">
      <joint name="passive" type="hinge" axis="1 0 0"/>
      <geom name="b_visual" type="sphere" size="0.01" density="1000"
            group="2" contype="0" conaffinity="0" rgba="0 0 0 1"/>
    </body>
  </body>
</worldbody>
<actuator>
  <position name="driver_act" joint="driver" kp="10"/>
  <position name="passive_act" joint="passive" kp="10"/>
</actuator>
</mujoco>""")


def _cfg(link_a="a"):
    return {
        "closures": [{
            "link_a": link_a,
            "link_b": "b",
            "point": [0.0, 0.0, 0.0],
            "axis": [1.0, 0.0, 0.0],
        }],
        "dependent": ["passive"],
        "independent": ["driver"],
    }


def _write_primitive_export_pkg(tmp_path):
    """Real exporter input: no CAD/mesh dependency, just primitive URDF shapes."""
    urdf_dir = tmp_path / "urdf"
    urdf_dir.mkdir(parents=True)

    def link(name):
        return f"""<link name="{name}">
  <inertial><origin xyz="0 0 0"/><mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
  <visual><geometry><box size="0.04 0.04 0.04"/></geometry></visual>
  <collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision>
</link>"""

    text = f"""<?xml version="1.0"?>
<robot name="robot">
{link('base_link')}
{link('a')}
{link('b')}
<joint name="driver" type="revolute">
  <origin xyz="0.1 0 0"/><parent link="base_link"/><child link="a"/>
  <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="2" velocity="4"/>
</joint>
<joint name="passive" type="revolute">
  <origin xyz="-0.1 0 0"/><parent link="base_link"/><child link="b"/>
  <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="2" velocity="4"/>
</joint>
</robot>
"""
    (urdf_dir / "robot.urdf").write_text(text, encoding="utf-8")
    (tmp_path / "loop_closures.yaml").write_text(
        yaml.safe_dump(_cfg(), sort_keys=False), encoding="utf-8")
    return tmp_path


def test_closed_loop_becomes_two_connects_and_passive_actuator_is_removed(tmp_path):
    _write_working_urdf(tmp_path)
    root = _mjcf_root()
    report = F._postprocess_root(
        root,
        pkg_dir=str(tmp_path),
        robot_name="robot",
        loop_closures=_cfg(),
        mjcf_mod=_FakeMjcf,
    )

    connects = root.findall("./equality/connect")
    assert len(connects) == 2
    assert {c.get("body1") for c in connects} == {"a"}
    assert {c.get("body2") for c in connects} == {"b"}

    p0 = np.asarray([float(x) for x in connects[0].get("anchor").split()])
    p1 = np.asarray([float(x) for x in connects[1].get("anchor").split()])
    assert np.linalg.norm(p1 - p0) == pytest.approx(
        F._WITNESS_SEPARATION_M, abs=1e-10)

    actuators = root.findall("./actuator/*")
    assert [a.get("joint") for a in actuators] == ["driver"]
    assert report["closure"]["closures"] == 1
    assert report["closure"]["connects"] == 2
    assert report["actuator"]["before"] == 2
    assert report["actuator"]["after"] == 1


def test_fixed_merged_closure_endpoint_maps_to_surviving_body(tmp_path):
    _write_working_urdf(tmp_path, fixed_endpoint=True)
    root = _mjcf_root()  # a_mount was merged into body a by the staged exporter
    report = F._postprocess_root(
        root,
        pkg_dir=str(tmp_path),
        robot_name="robot",
        loop_closures=_cfg(link_a="a_mount"),
        mjcf_mod=_FakeMjcf,
    )

    connects = root.findall("./equality/connect")
    assert len(connects) == 2
    assert all(c.get("body1") == "a" for c in connects)
    assert not report["closure"]["skipped"]


def test_visual_black_is_replaced_but_explicit_color_wins(tmp_path):
    _write_working_urdf(tmp_path)
    root = _mjcf_root()
    report = F._postprocess_root(
        root,
        pkg_dir=str(tmp_path),
        robot_name="robot",
        loop_closures=_cfg(),
        colors={"a": "#ff8000"},
        mjcf_mod=_FakeMjcf,
    )

    a = root.find(".//body[@name='a']/geom")
    b = root.find(".//body[@name='b']/geom")
    assert a.get("rgba") == "1 0.501961 0 1"
    assert b.get("rgba") == F._DEFAULT_VISUAL_RGBA
    assert report["visual_colors_fixed"] == 2


def test_no_loop_sidecar_keeps_all_actuators(tmp_path):
    _write_working_urdf(tmp_path)
    root = _mjcf_root()
    report = F._postprocess_root(
        root,
        pkg_dir=str(tmp_path),
        robot_name="robot",
        loop_closures=None,
        mjcf_mod=_FakeMjcf,
    )

    assert root.find("equality") is None
    assert [a.get("joint") for a in root.findall("./actuator/*")] == [
        "driver", "passive"
    ]
    assert report["actuator"]["after"] == 2


def test_public_in_memory_export_reads_sidecar_and_restores_loop(tmp_path):
    from sw2robot.exporter.mjcf_export import build_mjcf_package

    pkg_dir = _write_primitive_export_pkg(tmp_path / "src")
    pkg, files = build_mjcf_package(
        str(pkg_dir), "robot", floating_base=False,
        foot_contacts=False, imu_sensors=False,
    )
    assert pkg == "robot_mjcf"
    xml = next(data for arc, data in files if arc.endswith("/mjcf/robot.xml"))
    root = ET.fromstring(xml)
    assert len(root.findall("./equality/connect")) == 2
    assert [a.get("joint") for a in root.findall("./actuator/*")] == ["driver"]


def test_postprocessed_closed_loop_compiles_in_mujoco_when_available(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    _write_working_urdf(tmp_path)
    root = _mjcf_root()
    F._postprocess_root(
        root,
        pkg_dir=str(tmp_path),
        robot_name="robot",
        loop_closures=_cfg(),
        mjcf_mod=_FakeMjcf,
    )

    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    assert model.neq == 2
    assert model.nu == 1
