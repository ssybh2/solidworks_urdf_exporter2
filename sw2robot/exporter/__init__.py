"""SolidWorks / Autodesk Inventor -> URDF, ROS and MuJoCo exporters."""

from .actuator_fixes import install as _install_actuator_fixes
from .appearance_fixes import install as _install_appearance_fixes
from .exporter_fixes import install as _install_exporter_fixes

# The CAD graph can contain closed kinematic loops while URDF is necessarily a
# tree. Install the MJCF post-process first so BOTH CLI exports and the web ZIP
# exporter restore loop equality constraints and passive joints consistently.
_install_exporter_fixes()

# Inventor's effective occurrence colours are captured during extraction and
# persisted beside graph.json. Layer them into working URDF / detached ROS/MJCF
# exports outside the closed-loop wrappers; explicit editor colours still win.
_install_appearance_fixes()

# Finally allow an explicit actuator allow-list (or ACT_* naming convention) to
# prune any remaining movable-but-passive joints. This wrapper is intentionally
# outermost, after the closed-loop automatic dependent-joint filtering.
_install_actuator_fixes()

del _install_exporter_fixes, _install_appearance_fixes, _install_actuator_fixes
