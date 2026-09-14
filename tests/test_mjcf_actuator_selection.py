import xml.etree.ElementTree as ET

from sw2robot.exporter import actuator_fixes as A


def _root():
    return ET.fromstring("""<mujoco>
<worldbody><body name="base">
  <body name="a"><joint name="ACT_HIP_L" type="hinge"/></body>
  <body name="b"><joint name="PASS_KNEE_L" type="hinge"/></body>
  <body name="c"><joint name="wheel_raw" type="hinge"/></body>
</body></worldbody>
<actuator>
  <position name="a1" joint="ACT_HIP_L"/>
  <position name="a2" joint="PASS_KNEE_L"/>
  <position name="a3" joint="wheel_raw"/>
</actuator>
</mujoco>""")


def test_act_prefix_becomes_explicit_actuator_allowlist():
    root = _root()
    report = A._prune_actuators(root)
    assert report["before"] == 3
    assert report["after"] == 1
    assert [x.get("joint") for x in root.findall("./actuator/*")] == ["ACT_HIP_L"]


def test_configured_actuated_joints_apply_joint_rename(tmp_path):
    (tmp_path / "robot.joints.yaml").write_text(
        """actuated_joints:
  - wheel_internal
joint_names:
  wheel_internal: wheel_raw
""",
        encoding="utf-8",
    )
    selected = A._configured_actuated_joints(str(tmp_path), "robot")
    assert selected == {"wheel_raw"}
    root = _root()
    report = A._prune_actuators(root, selected)
    assert report["after"] == 1
    assert root.find("./actuator/*").get("joint") == "wheel_raw"


def test_no_explicit_or_act_prefix_leaves_existing_result_unchanged():
    root = ET.fromstring("""<mujoco><worldbody><body name="base">
<body name="a"><joint name="hip" type="hinge"/></body>
<body name="b"><joint name="knee" type="hinge"/></body>
</body></worldbody><actuator>
<position name="a" joint="hip"/><position name="b" joint="knee"/>
</actuator></mujoco>""")
    report = A._prune_actuators(root)
    assert report["before"] == report["after"] == 2
