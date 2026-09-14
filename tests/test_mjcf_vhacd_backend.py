from sw2robot.exporter.mjcf_vhacd_backend import _safe_mjcf_kwargs


def test_normal_mjcf_export_uses_vhacd_not_coacd():
    kwargs = _safe_mjcf_kwargs({})
    assert kwargs["strict_mesh_collision"] is False
    assert kwargs["collision"] == "vhacd"
    assert kwargs["coacd_quality"] == "fine"


def test_accidental_coacd_request_is_redirected_away_from_native_dll():
    kwargs = _safe_mjcf_kwargs({
        "collision": "coacd",
        "coacd_quality": "balanced",
    })
    assert kwargs["collision"] == "vhacd"
    assert kwargs["coacd_quality"] == "balanced"


def test_native_coacd_remains_explicitly_opt_in():
    kwargs = _safe_mjcf_kwargs({
        "collision": "coacd",
        "coacd_quality": "fine",
        "allow_native_coacd": True,
    })
    assert kwargs["collision"] == "coacd"
    assert "allow_native_coacd" not in kwargs


def test_explicit_non_coacd_collision_mode_is_preserved():
    kwargs = _safe_mjcf_kwargs({
        "strict_mesh_collision": False,
        "collision": "copy",
    })
    assert kwargs["collision"] == "copy"
    assert kwargs["strict_mesh_collision"] is False
