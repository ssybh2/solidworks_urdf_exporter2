# Autodesk Inventor backend

The `inventor-support` branch adds a Windows Autodesk Inventor extraction path while keeping sw2robot's CAD-independent editor/build pipeline.

## Data flow

```text
Inventor .iam / .ipt
        |
        | Inventor COM API
        v
 graph.json + meshes/*.stl + appearance_colors.yaml
        |
        +--> sw2robot browser editor
        |
        +--> working URDF + loop_closures.yaml
        |
        +--> ROS / ROS2 description package
        |
        `--> MuJoCo MJCF package
```

Inventor is needed only for extraction. After `graph.json` and meshes have been written, editing and export are CAD-independent.

## Windows installation

```powershell
git clone -b inventor-support https://github.com/ssybh2/solidworks_urdf_exporter2.git
cd solidworks_urdf_exporter2
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -e .
```

Autodesk Inventor must be installed on the Windows machine used for extraction.

Start the Inventor-aware browser editor with:

```powershell
.\.venv\Scripts\sw2robot-web.exe
```

Then open `http://localhost:8090` and select an `.iam` or `.ipt` file.

## Command-line workflow

Generate the normal sw2robot package and working URDF:

```powershell
.\.venv\Scripts\inv2robot.exe C:\CAD\robot.iam -o C:\CAD\robot_export
```

Generate MuJoCo MJCF as well:

```powershell
.\.venv\Scripts\inv2robot.exe C:\CAD\robot.iam -o C:\CAD\robot_export --mujoco
```

For a robot bolted to the world:

```powershell
.\.venv\Scripts\inv2robot.exe C:\CAD\robot.iam -o C:\CAD\robot_export --mujoco --mujoco-fixed-base
```

Reuse an already-running Inventor instance:

```powershell
.\.venv\Scripts\inv2robot.exe C:\CAD\robot.iam --attach --mujoco
```

Extract only, then edit later without Inventor:

```powershell
.\.venv\Scripts\inv2robot.exe C:\CAD\robot.iam -o C:\CAD\robot_export --extract-only
.\.venv\Scripts\sw2robot-web.exe
```

A single `.ipt` is accepted and becomes a one-link body.

## Recommended Inventor authoring style

For robot kinematics prefer Inventor **Assembly Joints** over relying only on legacy constraints:

| Inventor | sw2robot / URDF / MuJoCo |
| --- | --- |
| Rigid | fixed |
| Rotational | revolute / hinge |
| Slider | prismatic / slide |
| Locked | fixed |

Explicit Inventor Assembly Joints are authoritative. When explicit movable joints are present, old Insert/legacy constraints are not allowed to manufacture extra movable DOFs.

For a pin joint, define one clean **Rotational Joint** with the origin on the real pin/shaft axis. Avoid describing the same physical hinge with both a Planar Joint and a Rotational Joint.

Give useful frames named **UCS** objects, for example:

```text
UCS_IMU
UCS_CAMERA
UCS_TCP
UCS_FOOT_FL
```

These frames are imported into sw2robot and can later be used as robot ports/sites/sensor frames.

## What the Inventor extractor preserves

- top-level `.iam` component occurrences
- component local-to-assembly transforms
- grounded / hidden occurrence state
- native mass, center of mass and inertia tensor
- per-part binary STL explicitly exported in metres
- assembly and component UCS frames
- effective per-occurrence Inventor appearance colour
- native Assembly Joints and enabled travel limits
- stable ASCII internal link IDs even when the CAD occurrence names are Chinese

The browser UI still shows the original Inventor/Chinese names; the ASCII IDs are kept internally for URDF/ROS/MuJoCo compatibility.

## Unit handling

Inventor database units use centimetres for length, radians for angles and kilograms for mass. The backend converts:

- positions / translations: cm -> m
- inertia: kg*cm^2 -> kg*m^2
- STL export units: metre
- angular quantities: radians (no conversion)

Inventor `.stl` files are treated as metre-native by the shared mesh-inertia fallback, so density edits do not accidentally apply the SolidWorks millimetre scale a second time.

## Closed-loop mechanisms

A wheel-leg, four-bar or other closed linkage cannot be represented directly by standard URDF because URDF requires a tree. sw2robot therefore keeps two pieces of information:

```text
working URDF                  legal kinematic tree
loop_closures.yaml            movable edge(s) cut from that tree
```

The cut edge is not discarded.

### Browser editor

The Web editor loads `loop_closures.yaml` and runs a hard closed-loop IK solve. A user-driven joint is prescribed and the passive joints are solved so the closure is satisfied. If the requested driver value is outside the current assembly branch's reachable set, the pose is rejected/clipped instead of allowing the mechanism to tear apart.

### ROS / ROS2

The URDF remains a tree. ROS2 exports with detected closures additionally ship the closure configuration and the runtime `loop_closure_relay` helper so visualization can reconstruct the loop-dependent joint motion.

### MuJoCo MJCF

MuJoCo can represent a physical closed loop natively. The MJCF exporter restores every supported dropped revolute edge as **two collinear `equality/connect` constraints** between the two bodies. Two point-connect constraints remove the relative translation and axis-tilt DOFs while preserving rotation about the common hinge axis.

Closed-loop dependent/passive joints do not receive actuators. The independent tree coordinates remain actuated. This fixes the old behavior where every URDF hinge automatically became a MuJoCo actuator.

The exporter also remaps closure endpoints through fixed-link merging, so a closure whose original URDF endpoint was merged into a rigid parent still references the surviving MJCF body.

### Selecting exactly the active motors

A movable joint and an actuated joint are not the same thing. Passive linkage pins must stay revolute/hinge joints but should not receive a MuJoCo actuator.

For an exact final actuator list, add a top-level `actuated_joints:` list to the robot's `.joints.yaml`:

```yaml
actuated_joints:
  - hip_fl_joint
  - hip_fr_joint
  - hip_rl_joint
  - hip_rr_joint
  - wheel_l_joint
  - wheel_r_joint
```

`joint_names:` renames are applied before matching the final MJCF joint IDs, so the list may use the original internal joint IDs.

As a naming-only alternative, if any final joint name begins with `ACT_`, the MJCF exporter treats the complete `ACT_*` set as the actuator allow-list. A useful convention is:

```text
ACT_HIP_FL
ACT_HIP_FR
ACT_HIP_RL
ACT_HIP_RR
ACT_WHEEL_L
ACT_WHEEL_R

PASS_KNEE_L
PASS_KNEE_R
```

If neither `actuated_joints:` nor the `ACT_` convention is used, the existing closed-loop independent/dependent classification remains the default and no additional pruning is performed.

## Appearance export

Inventor extraction writes:

```text
appearance_colors.yaml
```

using the effective occurrence appearance. Those colours are automatically applied to:

- the working URDF where it has no explicit material colour;
- detached ROS exports;
- MuJoCo visual geoms.

Explicit editor / `joints.yaml` colour overrides win over the native CAD colour. If an older converter still emits a pure-black MuJoCo visual without a colour source, the MJCF post-processor replaces it with a neutral visible fallback.

## Current limitations

- Cylindrical, Planar and Ball Inventor Assembly Joints are multi-DOF. The current one-DOF tree model still treats these conservatively; use explicit Rigid/Rotational/Slider joints for the robot skeleton where possible.
- Flexible/moving sub-assembly internals are not yet recursively expanded by the Inventor backend. For now expose important moving links as top-level occurrences when possible.
- Inventor Model States / iAssembly variants are not yet exposed as sw2robot configurations.
- Legacy Mate/Flush/Angle/Tangent constraint inference is intentionally conservative compared with explicit Assembly Joints.
- Standard URDF itself remains a tree. Native physical loop closure is restored only by consumers that support it (the Web solver, ROS2 relay, and MuJoCo equality constraints).
- The source-installed `sw2robot-web` command is the currently recommended Inventor path. The standalone PyInstaller target still needs to be switched to the Inventor-aware wrapper before an Inventor-enabled binary release is published.
- The real Inventor extraction/browser path has been exercised on a representative Windows assembly. The newest native MuJoCo equality/actuator exporter changes should still be re-exported and validated on that same real robot before this branch is treated as production-ready.

## Quick validation after MuJoCo export

With the official Python MuJoCo package installed:

```powershell
$model = "C:\path\to\robot.xml"
$env:MJCF_MODEL = $model
python -c "import os,mujoco; m=mujoco.MjModel.from_xml_path(os.environ['MJCF_MODEL']); print('njnt=',m.njnt,'nu=',m.nu,'neq=',m.neq)"
python -m mujoco.viewer --mjcf="$model"
```

For a closed-loop robot, `neq` should be non-zero. The number of actuators `nu` should match the independent/active coordinates rather than every passive hinge in the URDF tree.
