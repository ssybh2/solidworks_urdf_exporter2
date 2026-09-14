from __future__ import annotations

import textwrap

import yaml

from sw2robot.editor.actuation_webserver import (
    actuation_payload,
    set_actuated_joint,
)


def _package(tmp_path):
    pkg = tmp_path / "robot"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "graph.json").write_text("{}", encoding="utf-8")
    (pkg / "urdf" / "robot.urdf").write_text(
        textwrap.dedent(
            """\
            <robot name="robot">
              <joint name="hip_motor" type="revolute">
                <parent link="base_link"/><child link="upper"/>
              </joint>
              <joint name="knee" type="revolute">
                <parent link="upper"/><child link="lower"/>
              </joint>
              <joint name="sensor_mount" type="fixed">
                <parent link="lower"/><child link="sensor"/>
              </joint>
            </robot>
            """
        ),
        encoding="utf-8",
    )
    # Automatic closed-loop state: hip is the driver, knee is passive.
    (pkg / "loop_closures.yaml").write_text(
        textwrap.dedent(
            """\
            closures:
              - link_a: upper
                link_b: lower
                point: [0, 0, 0]
                axis: [1, 0, 0]
            dependent: [knee]
            independent: [hip_motor]
            """
        ),
        encoding="utf-8",
    )
    # hip_motor is the displayed/final name; hip is the stable pre-rename ID.
    (pkg / "robot.joints.yaml").write_text(
        "joint_names:\n  hip: hip_motor\n",
        encoding="utf-8",
    )
    return pkg


def test_first_toggle_seeds_from_effective_auto_selection(tmp_path):
    pkg = _package(tmp_path)

    before = actuation_payload(str(pkg), "urdf/robot.urdf")
    assert before["mode"] == "auto"
    assert before["actuated"] == ["hip_motor"]
    assert before["passive"] == ["knee"]

    after = set_actuated_joint(
        str(pkg), "urdf/robot.urdf", "knee", True)
    assert after["mode"] == "configured"
    assert after["actuated"] == ["hip_motor", "knee"]

    cfg = yaml.safe_load(
        (pkg / "robot.joints.yaml").read_text(encoding="utf-8"))
    # Persist the stable key for a renamed joint, so later renames keep the
    # actuator attached to the same physical edge.
    assert cfg["actuated_joints"] == ["hip", "knee"]


def test_passive_toggle_can_explicitly_select_zero_motors(tmp_path):
    pkg = _package(tmp_path)
    set_actuated_joint(str(pkg), "urdf/robot.urdf", "knee", True)
    set_actuated_joint(str(pkg), "urdf/robot.urdf", "hip_motor", False)
    final = set_actuated_joint(str(pkg), "urdf/robot.urdf", "knee", False)

    assert final["mode"] == "configured"
    assert final["actuated"] == []
    assert final["passive"] == ["hip_motor", "knee"]

    cfg = yaml.safe_load(
        (pkg / "robot.joints.yaml").read_text(encoding="utf-8"))
    assert cfg["actuated_joints"] == []


def test_fixed_joint_cannot_be_marked_motor(tmp_path):
    pkg = _package(tmp_path)
    try:
        set_actuated_joint(
            str(pkg), "urdf/robot.urdf", "sensor_mount", True)
    except ValueError as exc:
        assert "not a movable joint" in str(exc)
    else:
        raise AssertionError("fixed joint unexpectedly accepted as Motor")
