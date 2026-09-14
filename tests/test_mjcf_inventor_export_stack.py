import xml.etree.ElementTree as ET

import yaml


def _link(name):
    return f"""<link name="{name}">
  <inertial><mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
  <visual><geometry><box size="0.04 0.04 0.04"/></geometry></visual>
  <collision><geometry><box size="0.04 0.04 0.04"/></geometry></collision>
</link>"""


def _package(tmp_path):
    urdf_dir = tmp_path / "urdf"
    urdf_dir.mkdir(parents=True)
    urdf = f"""<robot name="robot">
{_link('base_link')}
{_link('driver_link')}
{_link('passive_link')}
<joint name="driver" type="revolute">
  <origin xyz="0.1 0 0"/><parent link="base_link"/><child link="driver_link"/>
  <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="2" velocity="4"/>
</joint>
<joint name="passive" type="revolute">
  <origin xyz="-0.1 0 0"/><parent link="base_link"/><child link="passive_link"/>
  <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="2" velocity="4"/>
</joint>
</robot>"""
    (urdf_dir / "robot.urdf").write_text(urdf, encoding="utf-8")
    (tmp_path / "loop_closures.yaml").write_text(
        yaml.safe_dump({
            "closures": [{
                "link_a": "driver_link", "link_b": "passive_link",
                "point": [0.0, 0.0, 0.0], "axis": [1.0, 0.0, 0.0],
            }],
            "dependent": ["passive"],
            "independent": ["driver"],
        }, sort_keys=False),
        encoding="utf-8",
    )
    (tmp_path / "appearance_colors.yaml").write_text(
        yaml.safe_dump({"driver_link": "#2080E0"}), encoding="utf-8")
    return tmp_path


def test_public_mjcf_export_composes_loop_actuator_and_inventor_color(tmp_path):
    from sw2robot.exporter.mjcf_export import build_mjcf_package

    pkg_dir = _package(tmp_path / "src")
    _pkg, files = build_mjcf_package(
        str(pkg_dir), "robot", floating_base=False,
        foot_contacts=False, imu_sensors=False,
    )
    xml = next(data for arc, data in files if arc.endswith("/mjcf/robot.xml"))
    root = ET.fromstring(xml)

    assert len(root.findall("./equality/connect")) == 2
    assert [a.get("joint") for a in root.findall("./actuator/*")] == ["driver"]

    geom = root.find(".//body[@name='driver_link']/geom")
    assert geom is not None
    assert geom.get("rgba") == "0.12549 0.501961 0.878431 1"
