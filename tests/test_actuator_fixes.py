import xml.etree.ElementTree as ET

import yaml

from sw2robot.exporter.actuator_fixes import (
    _configured_actuated_joints,
    _loop_cfg_with_actuators,
    _prune_actuators,
)


def _root():
    return ET.fromstring("""<mujoco>
<worldbody>
  <body name="base">
    <body name="a"><joint name="ACT_HIP_L" type="hinge"/></body>
    <body name="b"><joint name="PASS_LINK" type="hinge"/></body>
    <body name="c"><joint name="wheel_joint" type="hinge"/></body>
  </body>
</worldbody>
<actuator>
  <position joint="ACT_HIP_L"/>
  <position joint="PASS_LINK"/>
  <position joint="wheel_joint"/>
</actuator>
</mujoco>""")


def test_act_prefix_is_a_safe_fallback_selector():
    root = _root()
    report = _prune_actuators(root)
    assert report["before"] == 3
    assert report["after"] == 1
    assert [a.get("joint") for a in root.findall("./actuator/*")] == ["ACT_HIP_L"]


def test_explicit_allow_list_overrides_prefix_convention():
    root = _root()
    _prune_actuators(root, {"ACT_HIP_L", "wheel_joint"})
    assert [a.get("joint") for a in root.findall("./actuator/*")] == [
        "ACT_HIP_L", "wheel_joint"
    ]


def test_configured_allow_list_follows_joint_rename(tmp_path):
    cfg = {
        "actuated_joints": ["old_hip", "wheel_joint"],
        "joint_names": {"old_hip": "ACT_HIP_L"},
    }
    (tmp_path / "robot.joints.yaml").write_text(
        yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
    assert _configured_actuated_joints(tmp_path, "robot") == {
        "ACT_HIP_L", "wheel_joint"
    }


def test_explicit_motor_can_override_automatic_loop_dependent_choice(tmp_path):
    cfg = {
        "closures": [{
            "link_a": "a", "link_b": "b",
            "point": [0, 0, 0], "axis": [1, 0, 0],
        }],
        "independent": ["auto_driver"],
        "dependent": ["physical_motor", "passive"],
    }
    out = _loop_cfg_with_actuators(
        str(tmp_path), cfg, {"physical_motor", "wheel_joint"})
    assert out["closures"] == cfg["closures"]
    assert out["independent"] == ["physical_motor", "wheel_joint"]
    assert out["dependent"] == ["passive"]
    # The source object is not mutated: other exporters still see the original
    # automatic IK split.
    assert cfg["independent"] == ["auto_driver"]
    assert cfg["dependent"] == ["physical_motor", "passive"]


def test_no_list_and_no_act_prefix_keeps_existing_automatic_result():
    root = ET.fromstring("""<mujoco>
<worldbody><body name="base"><body name="a"><joint name="j1"/></body></body></worldbody>
<actuator><position joint="j1"/></actuator>
</mujoco>""")
    report = _prune_actuators(root)
    assert report["before"] == report["after"] == 1
