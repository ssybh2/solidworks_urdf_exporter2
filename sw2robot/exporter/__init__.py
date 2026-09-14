"""SolidWorks / Autodesk Inventor -> URDF, ROS and MuJoCo exporters."""

from .appearance_fixes import install as _install_appearance_fixes
from .exporter_fixes import install as _install_exporter_fixes

# The CAD graph can contain closed kinematic loops while URDF is necessarily a
# tree. Install the MJCF post-process first so BOTH CLI exports and the web ZIP
# exporter restore loop equality constraints and passive joints consistently.
_install_exporter_fixes()

# Inventor's effective occurrence colours are captured during extraction and
# persisted beside graph.json. Layer them into detached ROS/MJCF exports outside
# the closed-loop wrappers; explicit editor colours still override CAD colour.
_install_appearance_fixes()

del _install_exporter_fixes, _install_appearance_fixes
