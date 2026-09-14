import yaml

from sw2robot.exporter.appearance_fixes import _load_native_colors, _merged_colors
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
