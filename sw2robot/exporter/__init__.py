"""SolidWorks / Autodesk Inventor -> URDF, ROS and MuJoCo exporters."""

from .actuator_fixes import install as _install_actuator_fixes
from .appearance_fixes import install as _install_appearance_fixes
from .exporter_fixes import install as _install_exporter_fixes
from .mjcf_validation_defaults import install as _install_mjcf_validation_defaults

# The CAD graph can contain closed kinematic loops while URDF is necessarily a
# tree. Install the MJCF post-process first so BOTH CLI exports and the web ZIP
# exporter restore loop equality constraints and passive joints consistently.
_install_exporter_fixes()

# Inventor's effective occurrence colours are captured during extraction and
# persisted beside graph.json. Layer them into working URDF / detached ROS/MJCF
# exports outside the closed-loop wrappers; explicit editor colours still win.
_install_appearance_fixes()

# Allow an explicit actuator allow-list (or ACT_* naming convention) to prune
# any remaining movable-but-passive joints after closed-loop driver selection.
_install_actuator_fixes()

# Outermost MuJoCo policy for mechanical-model validation: Motor interfaces stay
# present, but default damping is zero and collision stays on the source mesh;
# no synthetic foot-contact sphere is added unless a caller explicitly asks for
# one.  The underlying damping/contact/collision APIs remain opt-in interfaces.
_install_mjcf_validation_defaults()

del (
    _install_exporter_fixes,
    _install_appearance_fixes,
    _install_actuator_fixes,
    _install_mjcf_validation_defaults,
)
