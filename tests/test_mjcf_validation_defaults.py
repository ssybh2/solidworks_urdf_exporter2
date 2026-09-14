import xml.etree.ElementTree as ET

import pytest

from sw2robot.exporter.mjcf_validation_defaults import (
    _apply_motor_damping,
    _apply_selective_self_collision,
    _apply_spawn_height,
    _configured_spawn_height,
    _prepare_kwargs,
)


def test_validation_defaults_use_zero_damping_strict_mesh_and_selective_self_collision():
    kwargs, motor_damping, strip_default, selective_self_collision = _prepare_kwargs({})
    assert kwargs["backemf_damping"] is False
    assert kwargs["foot_contacts"] is False
    assert kwargs["collision"] == "copy"
    assert kwargs["self_collision"] is False
    assert motor_damping is None
    assert strip_default is True
    assert selective_self_collision is True


def test_strict_mesh_default_overrides_approximate_collision_mode():
    kwargs, _motor_damping, _strip_default, selective = _prepare_kwargs({
        "collision": "box",
    })
    assert kwargs["collision"] == "copy"
    assert selective is True


def test_explicit_legacy_self_collision_bool_is_preserved():
    kwargs, _motor_damping, _strip_default, selective = _prepare_kwargs({
        "self_collision": True,
    })
    assert kwargs["self_collision"] is True
    assert selective is False

    kwargs, _motor_damping, _strip_default, selective = _prepare_kwargs({
        "self_collision": False,
    })
    assert kwargs["self_collision"] is False
    assert selective is False


def test_explicit_selective_self_collision_overrides_legacy_bool():
    kwargs, _motor_damping, _strip_default, selective = _prepare_kwargs({
        "self_collision": True,
        "selective_self_collision": True,
    })
    # Selective mode asks the converter for its stable collision geometry, then
    # applies the pair policy in the final MJCF post-process.
    assert kwargs["self_collision"] is False
    assert selective is True


def test_explicit_damping_and_advanced_collision_interfaces_are_preserved():
    kwargs, motor_damping, strip_default, selective = _prepare_kwargs({
        "backemf_damping": True,
        "foot_contacts": True,
        "strict_mesh_collision": False,
        "collision": "coacd",
        "motor_damping": 0.25,
    })
    assert kwargs["backemf_damping"] is True
    assert kwargs["foot_contacts"] is True
    assert kwargs["collision"] == "coacd"
    assert "strict_mesh_collision" not in kwargs
    assert motor_damping == 0.25
    assert strip_default is False
    assert selective is True


def test_selective_self_collision_enables_remote_links_and_excludes_connections():
    root = ET.fromstring(
        '<mujoco>'
        '<worldbody>'
        '<geom name="ground" type="plane" contype="1" conaffinity="1"/>'
        '<body name="base">'
        '<geom name="base_collision" group="3" type="mesh"/>'
        '<body name="left">'
        '<geom name="left_collision" group="3" type="mesh"/>'
        '<body name="left_tip">'
        '<geom name="left_tip_collision" group="3" type="mesh"/>'
        '</body></body>'
        '<body name="right">'
        '<geom name="right_collision" group="3" type="mesh"/>'
        '</body>'
        '</body>'
        '</worldbody>'
        '<equality>'
        '<connect name="loop0" body1="left_tip" body2="right" anchor="0 0 0"/>'
        '</equality>'
        '</mujoco>'
    )

    changed, report = _apply_selective_self_collision(root)
    assert changed is True
    assert report["collision_geoms"] == 4

    collision_geoms = [
        g for g in root.iter("geom") if g.get("group") == "3"
    ]
    assert all(g.get("contype") == "2" for g in collision_geoms)
    assert all(g.get("conaffinity") == "3" for g in collision_geoms)

    ground = root.find("worldbody/geom")
    assert ground.get("contype") == "1"
    assert ground.get("conaffinity") == "1"

    excluded = {
        tuple(sorted((e.get("body1"), e.get("body2"))))
        for e in root.findall("contact/exclude")
    }
    # Direct tree-joint interfaces are excluded.
    assert ("base", "left") in excluded
    assert ("base", "right") in excluded
    assert ("left", "left_tip") in excluded
    # The dropped CAD loop hinge is also excluded from contact.
    assert ("left_tip", "right") in excluded
    # Siblings are NOT excluded: they must be able to collide instead of
    # passing through each other.
    assert ("left", "right") not in excluded


def test_selective_self_collision_is_idempotent():
    root = ET.fromstring(
        '<mujoco><worldbody><body name="a">'
        '<geom group="3" type="mesh"/><body name="b">'
        '<geom group="3" type="mesh"/></body></body></worldbody></mujoco>'
    )
    changed, _ = _apply_selective_self_collision(root)
    assert changed is True
    changed, _ = _apply_selective_self_collision(root)
    assert changed is False
    assert len(root.findall("contact/exclude")) == 1


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


def _spawn_model():
    return ET.fromstring(
        '<mujoco><worldbody><body name="base" pos="0 0 0">'
        '<freejoint name="floating_base"/></body></worldbody>'
        '<keyframe><key name="home" '
        'qpos="0 0 0.189986492 1 0 0 0 0 0"/></keyframe></mujoco>'
    )


def test_auto_spawn_height_copies_mesh_derived_home_to_default_pose():
    root = _spawn_model()
    changed, height = _apply_spawn_height(root, None)
    assert changed is True
    assert height == pytest.approx(0.189986492)
    assert root.find("worldbody/body").get("pos") == "0 0 0.189986492"
    # Auto must not rewrite the converter's home keyframe.
    assert root.find("keyframe/key").get("qpos").split()[2] == "0.189986492"


def test_custom_spawn_height_updates_default_pose_and_home_keyframe():
    root = _spawn_model()
    changed, height = _apply_spawn_height(root, 0.42)
    assert changed is True
    assert height == pytest.approx(0.42)
    assert root.find("worldbody/body").get("pos") == "0 0 0.42"
    assert root.find("keyframe/key").get("qpos").split()[2] == "0.42"


def test_spawn_height_does_not_affect_fixed_base_model():
    root = ET.fromstring(
        '<mujoco><worldbody><body name="base" pos="0 0 0"/></worldbody>'
        '<keyframe><key name="home" qpos="0 0"/></keyframe></mujoco>'
    )
    assert _apply_spawn_height(root, 0.5) == (False, None)
    assert root.find("worldbody/body").get("pos") == "0 0 0"


def test_spawn_height_config_auto_and_custom(tmp_path):
    assert _configured_spawn_height(str(tmp_path), "robot") is None
    (tmp_path / "robot.joints.yaml").write_text(
        "mujoco_spawn_height: 0.35\n", encoding="utf-8"
    )
    assert _configured_spawn_height(str(tmp_path), "robot") == pytest.approx(0.35)


def test_invalid_spawn_height_config_is_rejected(tmp_path):
    (tmp_path / "robot.joints.yaml").write_text(
        "mujoco_spawn_height: -0.1\n", encoding="utf-8"
    )
    with pytest.raises(ValueError, match="spawn_height"):
        _configured_spawn_height(str(tmp_path), "robot")
