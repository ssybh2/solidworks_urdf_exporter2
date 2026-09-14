from __future__ import annotations

import yaml

from sw2robot.editor.inventor_webserver import _loop_closure_payload


def test_loop_closure_payload_missing_sidecar_is_empty(tmp_path):
    assert _loop_closure_payload(tmp_path) == {
        "closures": [], "dependent": [], "independent": []
    }


def test_loop_closure_payload_reads_runtime_constraint_data(tmp_path):
    data = {
        "closures": [
            {
                "link_a": "calf_a",
                "link_b": "calf_b",
                "point": [0.1, -0.2, 0.3],
                "axis": [1.0, 0.0, 0.0],
            }
        ],
        "dependent": ["hip__calf_a", "calf_a__calf_b"],
        "independent": ["base_link__hip"],
    }
    (tmp_path / "loop_closures.yaml").write_text(
        yaml.safe_dump(data, sort_keys=False), encoding="utf-8")

    got = _loop_closure_payload(tmp_path)
    assert got == data
