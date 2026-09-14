import yaml

from sw2robot.exporter import appearance_fixes
from sw2robot.exporter.inventor_backend.extract import (
    _appearance_hex,
    _write_appearance_colors,
)


class _Color:
    Red = 12
    Green = 128
    Blue = 250


class _ColorValue:
    Value = _Color()


class _Appearance:
    def Item(self, name):
        assert name == "generic_diffuse"
        return _ColorValue()


class _Occurrence:
    Appearance = _Appearance()


class _PartDocument:
    ActiveAppearance = _Appearance()


def test_inventor_occurrence_generic_diffuse_becomes_hex():
    assert _appearance_hex(_Occurrence()) == "#0C80FA"


def test_inventor_part_active_appearance_becomes_hex():
    assert _appearance_hex(_PartDocument()) == "#0C80FA"


def test_appearance_sidecar_roundtrip_and_explicit_override(tmp_path):
    _write_appearance_colors(
        str(tmp_path),
        {"body": "#102030", "arm": "#ABCDEF"},
    )
    sidecar = tmp_path / "appearance_colors.yaml"
    assert sidecar.is_file()
    raw = yaml.safe_load(sidecar.read_text(encoding="utf-8"))
    assert raw == {"arm": "#ABCDEF", "body": "#102030"}

    assert appearance_fixes._load_native_colors(str(tmp_path)) == raw
    assert appearance_fixes._merged_colors(
        str(tmp_path), {"arm": "#FF0000", "wheel": "#00FF00"}
    ) == {
        "arm": "#FF0000",
        "body": "#102030",
        "wheel": "#00FF00",
    }


def test_empty_appearance_export_removes_stale_sidecar(tmp_path):
    path = tmp_path / "appearance_colors.yaml"
    path.write_text("body: '#ffffff'\n", encoding="utf-8")
    _write_appearance_colors(str(tmp_path), {})
    assert not path.exists()
