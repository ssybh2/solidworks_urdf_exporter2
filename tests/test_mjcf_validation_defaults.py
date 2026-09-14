import xml.etree.ElementTree as ET

import pytest

from sw2robot.exporter.mjcf_validation_defaults import (
    _apply_motor_damping,
    _prepare_kwargs,
)


def test_validation_defaults_use_zero_damping_and_source_mesh_collision():
    kwargs, motor_damping, strip_default = _prepare_kwargs({})
    assert kwargs["backemf_damping"] is False
    assert kwargs["foot_contacts"] is False
    assert kwargs["collision"] == "copy"
    assert motor_damping is None
    assert strip_default is True


def test_explicit_damping_and_collision_interfaces_are_preserved():
    kwargs, motor_damping, strip_default = _prepare_kwargs({
        "backemf_damping": True,
        "foot_contacts": True,
        "collision": "coacd",
        "motor_damping": 0.25,
    })
    assert kwargs["backemf_damping"] is True
    assert kwargs["foot_contacts"] is True
    assert kwargs["collision"] == "coacd"
    assert motor_damping == 0.25
    assert strip_default is False


def test_default_motor_damping_is_removed_without_deleting_motor_interface():
    root = ET.fromstring(
        '<mujoco><worldbody><body><joint name="motor" damping="3.18"/>'
        '<joint name="passive" damping="1.2"/></body></worldbody>'
        '<actuator><position joint="motor"/></actuator></mujoco>'
    )
    assert _apply_motor_damping(root, None, strip_default=True) is True
    motor = next(j for j in root.iter("joint") if j.get("name") == "motor")
    passive = next(j for j in root.iter("joint") if j.get("name") == "passive")
    assert motor.get("damping") is None
    assert passive.get("damping") == "1.2"
    assert root.find("actuator/position").get("joint") == "motor"


def test_motor_damping_scalar_and_mapping_are_available_as_opt_in_interfaces():
    root = ET.fromstring(
        '<mujoco><worldbody><body><joint name="m1"/><joint name="m2"/>'
        '<joint name="p"/></body></worldbody><actuator>'
        '<position joint="m1"/><position joint="m2"/></actuator></mujoco>'
    )
    assert _apply_motor_damping(root, 0.3, strip_default=True) is True
    joints = {j.get("name"): j for j in root.iter("joint")}
    assert joints["m1"].get("damping") == "0.3"
    assert joints["m2"].get("damping") == "0.3"
    assert joints["p"].get("damping") is None

    assert _apply_motor_damping(
        root, {"m2": 0.07}, strip_default=True) is True
    assert joints["m1"].get("damping") == "0.3"
    assert joints["m2"].get("damping") == "0.07"


def test_negative_motor_damping_is_rejected():
    root = ET.fromstring(
        '<mujoco><worldbody><body><joint name="m1"/></body></worldbody>'
        '<actuator><position joint="m1"/></actuator></mujoco>'
    )
    with pytest.raises(ValueError, match="damping"):
        _apply_motor_damping(root, -0.1, strip_default=True)
