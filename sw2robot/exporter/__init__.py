"""SolidWorks / Autodesk Inventor -> URDF, ROS and MuJoCo exporters."""

# The CAD graph can contain closed kinematic loops while URDF is necessarily a
# tree.  Install the MJCF post-process once at package import so BOTH CLI exports
# and the web ZIP exporter restore loop equality constraints and passive joints
# consistently.
from .exporter_fixes import install as _install_exporter_fixes

_install_exporter_fixes()
del _install_exporter_fixes

# Inventor's effective occurrence colours are captured during extraction and
# persisted beside graph.json.  Layer them into detached ROS/MJCF exports after
# the closed-loop wrappers above are installed; explicit editor colours still
# override the CAD appearance.
from .appearance_fixes import install as _install_appearance_fixes

_install_appearance_fixes()
del _install_appearance_fixes
