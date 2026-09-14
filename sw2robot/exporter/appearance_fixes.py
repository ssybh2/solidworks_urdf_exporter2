"""Propagate native CAD appearance colours through detached exporters.

Inventor extraction runs while the CAD application is available, but ROS/MJCF
exports are often created much later from the cached package.  The Inventor
backend therefore persists ``appearance_colors.yaml`` beside ``graph.json``.
This module merges those native colours into every portable export automatically.

Explicit editor / joints.yaml ``colors:`` values always win over CAD colours.
The internal URDF link IDs remain ASCII and unchanged; this only affects visual
appearance.
"""

from __future__ import annotations

from functools import wraps
import os

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


def install():
    """Patch ROS and MuJoCo public exporters once.

    ``exporter_fixes.install`` runs first and wraps MJCF for closed-loop equality
    constraints / passive-actuator filtering.  These wrappers sit outside that
    layer and only supply the merged colour map, so both fixes compose instead
    of replacing one another.
    """
    from . import mjcf_export as mjcf_mod
    from . import ros_export as ros_mod

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
