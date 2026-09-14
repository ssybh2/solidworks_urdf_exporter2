import xml.etree.ElementTree as ET

import yaml

from sw2robot.exporter.appearance_fixes import (
    _apply_working_urdf_colors,
    _load_native_colors,
    _merged_colors,
)
from sw2robot.exporter.inventor_backend.extract import _appearance_hex


class _Color:
    def __init__(self, r, g, b):
        self.Red = r
        self.Green = g
        self.Blue = b


class _AssetValue:
    def __init__(self, value):
        self.Value = value


class _Asset:
    def __init__(self, color):
        self._diffuse = _AssetValue(color)

    def Item(self, name):
        if name != "generic_diffuse":
            raise KeyError(name)
        return self._diffuse

    @property
    def Count(self):
        return 0


class _Occurrence:
    def __init__(self, color):
        self.Appearance = _Asset(color)


def test_inventor_effective_appearance_to_hex():
    assert _appearance_hex(_Occurrence(_Color(12, 128, 255))) == "#0C80FF"


def test_native_color_sidecar_and_explicit_override(tmp_path):
    path = tmp_path / "appearance_colors.yaml"
    path.write_text(
        yaml.safe_dump({"arm": "#112233", "wheel": "#445566"}),
        encoding="utf-8",
    )

    assert _load_native_colors(tmp_path) == {
        "arm": "#112233",
        "wheel": "#445566",
    }
    assert _merged_colors(tmp_path, {"arm": "#ABCDEF"}) == {
        "arm": "#ABCDEF",
        "wheel": "#445566",
    }


def test_malformed_native_colors_are_ignored(tmp_path):
    (tmp_path / "appearance_colors.yaml").write_text(
        yaml.safe_dump({"ok": "#AABBCC", "bad": "black", "empty": None}),
        encoding="utf-8",
    )
    assert _load_native_colors(tmp_path) == {"ok": "#AABBCC"}


def test_working_urdf_gets_native_color_without_overwriting_explicit_material(tmp_path):
    path = tmp_path / "robot.urdf"
    path.write_text(
        """<robot name="r">
<link name="arm">
  <visual><geometry><box size="1 1 1"/></geometry></visual>
</link>
<link name="wheel">
  <visual>
    <geometry><cylinder radius="1" length="1"/></geometry>
    <material name="explicit"><color rgba="1 0 0 1"/></material>
  </visual>
</link>
</robot>""",
        encoding="utf-8",
    )

    assert _apply_working_urdf_colors(
        path, {"arm": "#2080E0", "wheel": "#00FF00"}) == 1
    root = ET.parse(path).getroot()
    arm = root.find("./link[@name='arm']/visual/material/color")
    wheel = root.find("./link[@name='wheel']/visual/material/color")
    assert arm.get("rgba") == "0.12549 0.501961 0.878431 1"
    assert wheel.get("rgba") == "1 0 0 1"
