import xml.etree.ElementTree as ET

from sw2robot.exporter.actuator_fixes import (
    _set_startup_actuation,
    _startup_actuation_enabled,
)


def test_startup_actuation_defaults_disabled(tmp_path):
    assert _startup_actuation_enabled(str(tmp_path), "robot") is False

    root = ET.fromstring(
        '<mujoco><compiler/><option gravity="0 0 -9.81"/>'
        '<worldbody/><actuator><position joint="j1"/></actuator></mujoco>'
    )
    assert _set_startup_actuation(root, False) is True
    flag = root.find("option/flag")
    assert flag is not None
    assert flag.get("actuation") == "disable"
    # Disabling forces must not delete the actuator interface.
    assert root.find("actuator/position") is not None


def test_startup_actuation_can_be_enabled_without_losing_other_flags(tmp_path):
    (tmp_path / "robot.joints.yaml").write_text(
        "mujoco_actuation_enabled: true\n", encoding="utf-8"
    )
    assert _startup_actuation_enabled(str(tmp_path), "robot") is True

    root = ET.fromstring(
        '<mujoco><option><flag actuation="disable" contact="disable"/>'
        '</option><worldbody/></mujoco>'
    )
    assert _set_startup_actuation(root, True) is True
    flag = root.find("option/flag")
    assert flag is not None
    assert flag.get("actuation") is None
    assert flag.get("contact") == "disable"


def test_startup_actuation_disable_is_idempotent():
    root = ET.fromstring(
        '<mujoco><option><flag actuation="disable"/></option><worldbody/></mujoco>'
    )
    assert _set_startup_actuation(root, False) is False
    assert root.find("option/flag").get("actuation") == "disable"
