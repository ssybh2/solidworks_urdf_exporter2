"""Propagate native CAD appearance colours through every exporter.

Inventor extraction runs while the CAD application is available, but URDF,
ROS and MJCF exports are often created much later from the cached package. The
Inventor backend therefore persists ``appearance_colors.yaml`` beside
``graph.json``. This module merges those native colours into exports
automatically.

Explicit editor / joints.yaml ``colors:`` values always win where the exporter
accepts them. The internal URDF link IDs remain ASCII and unchanged; this only
affects visual appearance.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from functools import wraps

_APPEARANCE_SIDECAR = "appearance_colors.yaml"


def _load_native_colors(pkg_dir):
    """Return ``{final link id: '#RRGGBB'}`` from the CAD appearance sidecar."""
    path = os.path.join(os.path.abspath(pkg_dir), _APPEARANCE_SIDECAR)
    if not os.path.isfile(path):
        return {}
    try:
        import yaml

        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}
    if not isinstance(data, dict):
        return {}
    out = {}
    for key, value in data.items():
        if not key or not value:
            continue
        text = str(value).strip()
        if text.startswith("#") and len(text) in (7, 9):
            out[str(key)] = text
    return out


def _merged_colors(pkg_dir, explicit=None):
    """Native CAD colours with explicit web/YAML overrides layered on top."""
    out = _load_native_colors(pkg_dir)
    if isinstance(explicit, dict):
        out.update({str(k): str(v) for k, v in explicit.items() if k and v})
    return out


def _rgba_text(value):
    """``#RRGGBB[AA]`` -> URDF 0..1 ``rgba`` text, or None."""
    if not value:
        return None
    text = str(value).strip().lstrip("#")
    if len(text) == 6:
        text += "ff"
    if len(text) != 8:
        return None
    try:
        channels = [int(text[i:i + 2], 16) / 255.0 for i in (0, 2, 4, 6)]
    except ValueError:
        return None
    return " ".join(f"{v:.6g}" for v in channels)


def _apply_working_urdf_colors(urdf_path, colors):
    """Add native colour to uncoloured working-URDF visual blocks.

    Do not overwrite an existing ``<material><color>``: an imported URDF or a
    future explicit working-URDF material is more authoritative than Inventor's
    one-colour-per-occurrence fallback.
    """
    if not colors or not os.path.isfile(urdf_path):
        return 0
    try:
        tree = ET.parse(urdf_path)
    except (OSError, ET.ParseError):
        return 0
    changed = 0
    for link in tree.getroot().findall("link"):
        rgba = _rgba_text(colors.get(link.get("name")))
        if rgba is None:
            continue
        for visual in link.findall("visual"):
            material = visual.find("material")
            color = material.find("color") if material is not None else None
            if color is not None and color.get("rgba"):
                continue
            if material is None:
                material = ET.SubElement(visual, "material")
                material.set("name", f"{link.get('name')}_cad_appearance")
            if color is None:
                color = ET.SubElement(material, "color")
            color.set("rgba", rgba)
            changed += 1
    if changed:
        ET.indent(tree, space="  ")
        tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    return changed


def install():
    """Patch working URDF, ROS and MuJoCo public exporters once.

    ``exporter_fixes.install`` runs first and wraps MJCF for closed-loop equality
    constraints / passive-actuator filtering. These wrappers sit outside that
    layer and only supply the merged colour map, so both fixes compose instead
    of replacing one another.
    """
    from . import mjcf_export as mjcf_mod
    from . import ros_export as ros_mod
    from . import urdf_writer as urdf_mod

    if not getattr(urdf_mod, "_cad_native_colors_installed", False):
        original_urdf_write = urdf_mod.write_urdf

        @wraps(original_urdf_write)
        def write_urdf(model, path, *args, **kwargs):
            result = original_urdf_write(model, path, *args, **kwargs)
            # Working layout is <pkg>/urdf/<robot>.urdf.
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(path)))
            _apply_working_urdf_colors(path, _load_native_colors(pkg_dir))
            return result

        urdf_mod.write_urdf = write_urdf
        urdf_mod._cad_native_colors_installed = True

    if not getattr(ros_mod, "_cad_native_colors_installed", False):
        original_ros_build = ros_mod.build_ros_description

        @wraps(original_ros_build)
        def build_ros_description(*args, **kwargs):
            pkg_dir = args[0] if args else kwargs["pkg_dir"]
            kwargs["colors"] = _merged_colors(pkg_dir, kwargs.get("colors"))
            return original_ros_build(*args, **kwargs)

        ros_mod.build_ros_description = build_ros_description
        ros_mod._cad_native_colors_installed = True

    if not getattr(mjcf_mod, "_cad_native_colors_installed", False):
        # At this point these may already be exporter_fixes' closed-loop wrappers.
        # Wrap the CURRENT public functions so the equality/actuator fixes remain
        # in the call chain and receive the same merged colour map.
        original_write = mjcf_mod.write_mjcf_package
        original_build = mjcf_mod.build_mjcf_package

        @wraps(original_write)
        def write_mjcf_package(*args, **kwargs):
            pkg_dir = args[0] if args else kwargs["pkg_dir"]
            kwargs["colors"] = _merged_colors(pkg_dir, kwargs.get("colors"))
            return original_write(*args, **kwargs)

        @wraps(original_build)
        def build_mjcf_package(*args, **kwargs):
            pkg_dir = args[0] if args else kwargs["pkg_dir"]
            kwargs["colors"] = _merged_colors(pkg_dir, kwargs.get("colors"))
            return original_build(*args, **kwargs)

        mjcf_mod.write_mjcf_package = write_mjcf_package
        mjcf_mod.build_mjcf_package = build_mjcf_package
        mjcf_mod._cad_native_colors_installed = True
