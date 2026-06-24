# Scripts

User-facing commands live in `scripts/`. This page lists each one, what it does,
and its flags. Run any of them from the workspace root, e.g. `scripts/<name>`.

> Helper scripts under `scripts/helpers/` are internal building blocks called by
> `create-ros-pkg`/`regenerate-pkg` and are not meant to be run directly.

## Overview

| Script | Purpose |
|---|---|
| `create-ros-pkg` | Generate a ROS package (world, MoveIt config, launch) from a lab config |
| `regenerate-pkg` | Re-derive a generated package from its own `config/robots.json` |
| `add-model` | Install a 3D model and emit/append/attach a `static_objects` (or RABIT `model`) entry |
| `resize-model` | Resize an already-installed model (edits its `model.sdf`) |
| `translate-config` | Convert a RABIT/general-schema config to the flat `diff_robots` spec |
| `preview-world` | Open a Gazebo world of just the static objects (no arms/MoveIt) |
| `serve-docs` | Build and serve this documentation |
| `integration-tests` | Run the integration test suite |
| `launch-wrappers/*` | One-line wrappers that build + launch a specific demo package |
| `setup.sh` | Install dependencies and build the workspace |

---

## create-ros-pkg

Generates a complete ROS package — URDF/Xacro, MoveIt config, Gazebo world
(including `static_objects`), launch files — and builds it. Runs interactively by
default, or non-interactively from a config. The package is written to
`src/generated/<package_name>/`, keeping generated packages separate from the
hand-authored ones in `src/`.

```
scripts/create-ros-pkg [--no-build]
scripts/create-ros-pkg --from-json <file.json> [--no-build]
scripts/create-ros-pkg --rabit <file.json> [--no-build]
```

| Flag | What it does |
|---|---|
| *(none)* | Interactive prompts for package name, robots, poses, joints |
| `--from-json <file>` | Build from a flat `diff_robots`-style spec (see `configs/examples/simulator-schema/` and `configs/schemas/`) |
| `--rabit <file>` | Build from a general-schema (RABIT) config (see `configs/examples/rabit-schema/`); translated via `translate-config` first. Mutually exclusive with `--from-json` |
| `--no-build` | Generate the files only; skip the `colcon build` step |
| `-h`, `--help` | Show usage |

## regenerate-pkg

Re-derives all generated files in `src/generated/<pkg>/` from that package's
`config/robots.json` (edit the JSON, then regenerate). Leaves `package.xml`,
`CMakeLists.txt`, and `config/robots.json` itself untouched.

```
scripts/regenerate-pkg [--no-build] <package_name>
```

| Flag / arg | What it does |
|---|---|
| `<package_name>` | The package under `src/generated/` to regenerate |
| `--no-build` | Skip the `colcon build` step (e.g. to inspect the regenerated files) |

## add-model

Installs a 3D model into the shared model library
(`src/multi_arm_lab_sim_description/models/`), measures it (units, size, resting
Z, model.sdf scale), and emits a ready entry. The source is auto-detected: a
Gazebo Fuel URL, a local model folder, or a bare mesh file
(`.stl`/`.dae`/`.obj`/`.glb`/`.gltf`/…). glTF (`.glb`/`.gltf`) is auto-converted
to OBJ so Gazebo can render it.

The entry is **printed** by default; `--into` appends it to a flat config's
`static_objects`, while `--rabit-into` attaches a `model` block to an entry in a
RABIT (general-schema) config.

```
scripts/add-model <source> [--name N] [--scale F] [--models-dir DIR]
                  [--into CONFIG.json | --rabit-into RABIT.json --attach-to KEY]
```

| Flag / arg | What it does |
|---|---|
| `<source>` | Fuel URL, local model folder, or mesh file to install |
| `--name N` | Override the installed model name (slugified) |
| `--into CONFIG.json` | Append the generated entry to a flat config's `static_objects` |
| `--rabit-into RABIT.json` | Attach a `model` block to a RABIT config (needs `--attach-to`) |
| `--attach-to KEY` | `key` of the `passive_object` / action-device instance to attach to |
| `--scale F` | Install at `F`× the model's native size; edits the installed `model.sdf`, the single source of scale for both Gazebo and MoveIt |
| `--models-dir DIR` | Install location (default: the shared description package's `models/`) |

Scale is never written into the config: it lives in the model's `model.sdf`,
which Gazebo applies via `include` and `scene_publisher.py` reads for the MoveIt
collision mesh.

## resize-model

Resizes a model already installed under `models/`, multiplying its current size
by `factor`. It edits only the model's `model.sdf` `<scale>` — the single source
of scale that both Gazebo and MoveIt read — so no config edits are needed. Rebuild
`multi_arm_lab_sim_description` and relaunch afterward. (It does not move the
object; adjust the pose Z by hand if the new size changes the resting height.)

```
scripts/resize-model <name> <factor>
```

| Flag / arg | What it does |
|---|---|
| `<name>` | Model name under `models/` |
| `<factor>` | Multiply the current size by this |
| `--models-dir DIR` | Models location (default: the shared description package's `models/`) |

## translate-config

Translates a general-schema (RABIT) lab config into the flat `diff_robots` spec
that `create-ros-pkg --from-json` consumes. Used automatically by
`create-ros-pkg --rabit`; run directly to inspect the translation.

```
scripts/translate-config <source.json> <destination.json>
```

| Arg | What it does |
|---|---|
| `<source.json>` | The general-schema config to read |
| `<destination.json>` | Where to write the flat `diff_robots` spec |

## preview-world

Opens a Gazebo world built from just a config's static objects — no ROS package,
no MoveIt, no arms — for iterating on lab layout before a full `create-ros-pkg`
run. Accepts either a RABIT config or a flat `diff_robots` spec.

```
scripts/preview-world <config.json> [-o world.sdf] [--no-gui] [--print-only]
```

| Flag / arg | What it does |
|---|---|
| `<config.json>` | The lab config to preview |
| `-o world.sdf` | Write the generated world SDF to this path |
| `--no-gui` | Run the simulator server headless (no Gazebo GUI) |
| `--print-only` | Generate the world SDF and print its path; do not launch Gazebo |

## serve-docs

Builds this Sphinx documentation and serves it locally.

```
scripts/serve-docs [--port N] [--build-only] [--open]
```

| Flag | What it does |
|---|---|
| `--port N` | Port to serve on (default: 8000) |
| `--build-only` | Build the HTML and exit (don't start a server) |
| `--open` | Open the built docs in the default browser |

## integration-tests

Runs the integration test suite (`pytest test/integration`). Any extra arguments
are passed straight through to `pytest`.

```
scripts/integration-tests [pytest args...]
```

| Arg | What it does |
|---|---|
| `[pytest args...]` | Forwarded to `pytest` (e.g. `-k name`, `-x`, a specific test path) |

## launch-wrappers

Small convenience wrappers under `scripts/launch-wrappers/`. Each sources ROS and
the workspace overlay, then `ros2 launch`es one package — a shortcut for the demos
referenced elsewhere in these docs. Run them from the workspace root.

```
scripts/launch-wrappers/launch_<demo>
```

| Wrapper | What it launches |
|---|---|
| `launch_diff_robots` | The `diff_robots` two-arm demo |
| `launch_three_robots` | A three-arm demo |
| `launch_static_objects_demo` | A lab with primitive `static_objects` |
| `launch_mesh_objects_demo` | A lab with mesh-based static objects |
| `launch_rabit_lab` | A RABIT lab config (simulation) |
| `launch_rabit_real_lab` | The RABIT real-lab layout |
| `multiple-robots-moveit` | `two_arm_moveit_config` — controllers, Gazebo, RViz, MoveIt (forwards extra args) |
| `multiple-robots-with-controllers-no-moveit` | `multi_arm_lab_sim_bringup` — controllers + Gazebo, no MoveIt (forwards extra args) |

> The per-demo `launch_<demo>` wrappers run a full `colcon build` first, so the
> first launch after a change may take a while.

## setup.sh

Installs ROS 2 Humble, Gazebo, vendor robot packages, and builds the colcon
workspace. Run once when setting up the repo on a fresh machine.

```
scripts/setup.sh
```
