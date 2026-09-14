# Autodesk Inventor backend (experimental)

The `inventor-support` branch adds a Windows Inventor extraction path without replacing the existing sw2robot editor/build pipeline.

## Data flow

```text
Inventor .iam / .ipt
        |
        |  Inventor COM API
        v
 graph.json + meshes/*.stl
        |
        +--> existing sw2robot browser editor
        |
        +--> existing URDF exporter
        |
        `--> existing MuJoCo MJCF exporter
```

Inventor is only required for the extraction step. Once `graph.json` and the meshes exist, the normal CAD-independent sw2robot workflow is reused.

## Install from this branch

```powershell
git clone -b inventor-support https://github.com/ssybh2/solidworks_urdf_exporter2.git
cd solidworks_urdf_exporter2
py -3.12 -m pip install -e .
```

Autodesk Inventor must be installed on the Windows machine used for extraction.

## Browser workflow

The normal browser command is Inventor-aware on this branch:

```powershell
sw2robot-web
```

Open the file browser, navigate to an Inventor project folder, and select a `.iam` or `.ipt`. The server will:

1. try to attach to an already-running Inventor instance;
2. start a private Inventor automation instance if none is attachable;
3. extract `graph.json`, SI-unit mass/inertia, UCS frames and metre-scaled STL meshes;
4. build the normal working URDF;
5. open the result in the existing sw2robot editor.

After that point the existing joint editor, coordinate-frame tooling, collision tooling and URDF/MJCF export path are shared with the SolidWorks backend.

## Command-line workflow

Generate the normal sw2robot package and URDF:

```powershell
inv2robot C:\CAD\robot.iam -o C:\CAD\robot_export
```

Generate MuJoCo MJCF as well:

```powershell
inv2robot C:\CAD\robot.iam -o C:\CAD\robot_export --mujoco
```

For a robot bolted to the world:

```powershell
inv2robot C:\CAD\robot.iam -o C:\CAD\robot_export --mujoco --mujoco-fixed-base
```

If Inventor is already running, it can be reused:

```powershell
inv2robot C:\CAD\robot.iam --attach --mujoco
```

Extract only, then use the existing sw2robot browser/editor workflow on the generated package:

```powershell
inv2robot C:\CAD\robot.iam -o C:\CAD\robot_export --extract-only
sw2robot-web
```

A single `.ipt` file is also accepted and becomes a one-link robot/body.

## What is extracted in the first implementation

- top-level `.iam` component occurrences
- component local-to-assembly transforms
- grounded and hidden occurrence state
- native Inventor mass, center of mass, and inertia tensor
- per-part high-resolution binary STL, explicitly exported in metres
- assembly and component User Coordinate Systems (UCS)
- native Assembly Joints:
  - Rigid -> fixed
  - Rotational -> revolute
  - Slider -> prismatic
  - locked joints -> fixed
  - angular/linear travel limits when enabled
- classic Insert constraints as revolute joints
- remaining classic constraint relationships conservatively as fixed

The extracted joints are translated into sw2robot's existing `MateEdge` and `LimitJoint` intermediate representation. This is intentional: joint editing, axes, limits, the browser UI, URDF generation and MJCF generation continue to use the same code as the SolidWorks backend.

## Unit handling

Inventor database length is centimetres and mass is kilograms. The backend converts:

- positions / translations: cm -> m
- inertia: kg*cm^2 -> kg*m^2
- STL export units: metre
- angular quantities: radians (no conversion)

The shared mesh-inertia fallback also recognizes the Inventor `.stl` files as metre-native, so changing a density in the editor does not accidentally re-apply the SolidWorks millimetre scale.

## Current limitations

This is an MVP intended to get real Inventor robots into the existing editor quickly.

- Cylindrical, planar and ball Assembly Joints contain more than one DOF. sw2robot's current URDF-oriented intermediate model represents one joint DOF per tree edge, so these are kept fixed and reported as warnings for now.
- Flexible/moving sub-assembly internals are not yet recursively expanded. A top-level sub-assembly is currently one link.
- Classic Mate/Flush/Angle/Tangent constraint sets are not yet solved geometrically as deeply as the mature SolidWorks mate backend. A classic Insert constraint is recognized; other classic constraint pairs default to fixed and can be corrected in the existing editor.
- Inventor Model States / iAssembly variants are not yet exposed as sw2robot configurations.
- Mesh caching currently keys by source document path, not Model State.
- The source-installed `sw2robot-web` command uses the Inventor-aware wrapper. The existing PyInstaller `build_exe.py` still targets the upstream webserver entry point and needs a small follow-up before an Inventor-aware standalone `.exe` is released.
- This branch has unit tests for the CAD-independent conversions, but it still needs an end-to-end test on a real Inventor `.iam` installation/model before being considered production-ready.

## Recommended Inventor authoring style

For the cleanest automatic conversion, use Inventor **Assembly Joints** (Rigid, Rotational, Slider) for robot kinematics instead of relying only on a large collection of legacy assembly constraints. Give useful frames a named **UCS**. Those concepts map directly to robot links, joints, axes and frames and therefore need much less inference.
