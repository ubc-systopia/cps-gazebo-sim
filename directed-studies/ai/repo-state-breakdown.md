# `cps-gazebo-sim` — Repository State Breakdown

> **Snapshot date:** 2026-05-20
> **Branch:** `main` (up to date with `origin/main`)
> **Scope:** Whole repository, with a focused breakdown of `scripts/` and how the ROS 2 package generator is wired together.

---

## Table of Contents

1. [High-level Purpose](#high-level-purpose)
2. [Headline Capabilities](#headline-capabilities)
3. [Top-level Layout](#top-level-layout)
4. [Automated ROS Package Generation](#automated-ros-package-generation)
   - [Directory layout](#directory-layout)
   - [`create-ros-pkg` interactive flow](#create-ros-pkg-interactive-flow)
   - [`scripts/helpers/` reference](#scriptshelpers-reference)
   - [Conventions baked into the generator](#conventions-baked-into-the-generator)
5. [Collision Detection & Avoidance](#collision-detection--avoidance)
6. [`src/` Packages](#src-packages)
7. [Tests](#tests)
8. [Documentation](#documentation)
9. [Git State](#git-state)
10. [Headline Observations & Cleanup Targets](#headline-observations--cleanup-targets)

---

## High-level Purpose

A **ROS 2 Humble + Ignition Gazebo Fortress** workspace for simulating multi-arm robotics labs (Universal Robots family + MoveIt 2).

The headline capability is a **shell-driven package generator** that takes user input describing N robot arms and emits a fully wired MoveIt config package — URDF/XACRO, SRDF, controllers, RViz, launch — into `src/`.

| Target component | Version |
| --- | --- |
| OS | Ubuntu 22.04 (Jammy) |
| ROS | Humble Hawksbill |
| Simulator | Gazebo Fortress (Ignition) |
| Motion planning | MoveIt 2 |
| Python | 3.10 |
| CMake | ≥ 3.5 |

The working tree currently lives on macOS, but builds and runs are expected on Ubuntu.

---

## Headline Capabilities

The project today stands on two pillars:

### 1. Automated ROS package generation

`scripts/create-ros-pkg` takes interactive user input (number of robots, UR model per arm, base poses, initial joint values) and emits a complete, build-ready ROS 2 / MoveIt package into `src/`. The generated package includes URDF/XACRO, SRDF, MoveIt configs, controller YAMLs, an RViz layout, and a `demo.launch.py` that brings up Ignition Gazebo + MoveIt + RViz with controllers spawned per robot.

Rationale: the MoveIt Setup Assistant should not be in the loop for end-users configuring lab environments — the generator replaces that workflow with a single shell prompt.

### 2. Collision detection & avoidance

- **Avoidance** is set up automatically as part of package generation: the generator produces a per-package SRDF with MoveIt planning groups and the right self-collision / inter-arm collision disable rules so that motion planning rejects colliding trajectories from the start.
- **Detection** is exercised by an integration test (`test/integration/collision_tests/simple_collisions_test.py`) that drives the robots into overlapping joint states and verifies the collision-checking pipeline catches them.

See [Collision Detection & Avoidance](#collision-detection--avoidance) below for the detailed mechanics.

---

## Top-level Layout

```
cps-gazebo-sim/
├── scripts/              # Package-generation tooling (focus of this doc)
├── src/                  # ROS 2 packages (authored, vendored, and generated)
├── test/                 # integration/ (live) + other/ (graveyard)
├── docs/                 # Sphinx docs (guides/, quick_info/)
├── .repos/               # vcstool manifests for vendor robots
├── directed-studies/     # *untracked* — meeting notes (this file lives here)
├── setup.sh              # Host bootstrap (apt, ROS, Gazebo)
├── Makefile
├── requirements.txt / requirements.dev.txt
├── .pre-commit-config.yaml
├── .readthedocs.yaml
└── pytest.ini
```

`directed-studies/` is the only path appearing in `git status` — meeting notes for Apr 22, May 15, and May 22 2026.

---

## Automated ROS Package Generation

### Directory layout

```
scripts/
├── create-ros-pkg                            # Interactive entrypoint (~620 lines)
├── setup.sh                                  # Host bootstrap (locale, apt, ROS, Gazebo)
├── multiple-robots-moveit                    # Wraps: ros2 launch two_arm_moveit_config demo.launch.py
├── multiple-robots-with-controllers-no-moveit# Wraps: ros2 launch multi_arm_lab_sim_bringup multiple_ur.launch.py
├── integration-tests                         # pytest test/integration after sourcing overlays
└── helpers/                                  # 20 generator scripts + one XACRO template
```

| Top-level script | Role |
| --- | --- |
| `create-ros-pkg` | Interactive package generator — prompts, validates, fans out to helpers. |
| `setup.sh` | One-shot Ubuntu bootstrap (ROS apt source, `ros-humble-desktop`, Gazebo Fortress). |
| `multiple-robots-moveit` | Convenience launcher for the hand-built two-arm MoveIt demo. |
| `multiple-robots-with-controllers-no-moveit` | Convenience launcher for the no-MoveIt bringup demo. |
| `integration-tests` | Sources overlays and runs `pytest test/integration`. |

### `create-ros-pkg` interactive flow

1. **Prompt** for package name and robot count (1–10).
2. **For each robot**, prompt for:
   - UR model — one of `ur3`, `ur3e`, `ur5`, `ur5e`, `ur10`, `ur10e`, `ur16e`, `ur20`, `ur30`.
   - Base pose — `x`, `y`, `z` (m) and `roll`, `pitch`, `yaw` (rad).
   - Initial joint values — 6 floats: shoulder pan/lift, elbow, wrist 1/2/3 (rad).
3. **Display** a configuration summary; ask for `y/N` confirmation.
4. **Assemble** colon-delimited tuples (`robotSpec`) and dispatch to ~16 helper scripts under `scripts/helpers/`.
5. **Expand** the generated `<pkg>.urdf.xacro` → `<pkg>.urdf` via whichever is available: `xacro`, `ros2 run xacro xacro`, or `python3 -m xacro`.
6. **Write** a convenience launcher at `scripts/launch_<pkg>` that runs `colcon build && ros2 launch <pkg> demo.launch.py`.

`robotSpec` tuple format (used by several helpers):

```
key:model:x:y:z:R:P:Y:pan:lift:elbow:w1:w2:w3
```

### `scripts/helpers/` reference

#### Config-file generators → `<pkg>/config/`

| Helper | Output | Purpose |
| --- | --- | --- |
| `generate_robots_json.sh` | `robots.json` | Canonical per-robot metadata; consumed at runtime by `demo.launch.py` (`_read_robot_names`). |
| `generate_controllers.sh` | `controllers.yaml` | One `joint_trajectory_controller` per robot + `joint_state_broadcaster`. `update_rate: 100 Hz`. |
| `generate_moveit_controllers.sh` | `moveit_controllers.yaml` | MoveIt simple controller manager bindings. |
| `generate_joint_limits.sh` | `joint_limits.yaml` | Hardcoded UR-style limits (π for arm, 2π for wrists), scaling factor `0.1`. |
| `generate_kinematics.sh` | `kinematics.yaml` | `kdl_kinematics_plugin/KDLKinematicsPlugin` per robot group. |
| `generate_initial_positions.sh` | `initial_positions.yaml` | Per-joint starting positions. |
| `generate_pilz_cartesian_limits.sh` | `pilz_cartesian_limits.yaml` | Pilz planner velocity/acceleration limits. |
| `generate_sensors_3d.sh` | `sensors_3d.yaml` | **Copies from `src/two_arm_moveit_config/config/sensors_3d.yaml`**; falls back to `sensors: []`. |
| `generate_srdf.sh` | `<pkg>.srdf` | MoveIt group per robot (chain `base_link_inertia` → `tool0`), standard UR self-collision disables, pairwise base/arm cross-disables. |
| `generate_rviz_config.sh` | `<pkg>.rviz` | RViz layout with displays for each robot. |

#### URDF / XACRO generators → `<pkg>/urdf/`

| Helper | Output | Purpose |
| --- | --- | --- |
| `generate_pkg_xacro.sh` | `<pkg>.urdf.xacro` | **Main monolithic XACRO.** Single `ros2_control` block (`ign_ros2_control/IgnitionSystem`), per-robot `<xacro:ur_robot>` instances pulling from `multi_ur_description/config/<model>/*.yaml`, plus an `IgnitionROS2ControlPlugin` `<gazebo>` block pointing at `controllers.yaml`. |
| `generate_robot_description_template.sh` | `robot_description_template.xacro` | Near-duplicate of the above; consumed by the static-URDF helper. |
| `generate_robot_description_wrapper.sh` | `<pkg>_wrapper.urdf.xacro` | Modeled on `two_arm_moveit_config/config/mutli_arm.urdf.xacro` (sic — typo "mutli"). Macro instantiation intentionally commented out. |
| `generate_ros2_control_xacro.sh` | `<pkg>.ros2_control.xacro` | Defines a `multi_arm_ros2_control` xacro macro with per-robot joint blocks. |
| `generate_static_robot_description.sh` | `robot_description_template.urdf` | Expanded URDF snapshot of the template above. |

#### Launch / misc

| Helper | Output | Purpose |
| --- | --- | --- |
| `generate_demo_launch.sh` | `launch/demo.launch.py` | Builds MoveIt via `MoveItConfigsBuilder`; launches Ignition (`ros_gz_sim` with `-r empty.sdf`); spawns robot from `/robot_description`; runs RViz; spawns one controller per robot read dynamically from `robots.json` (+ `joint_state_broadcaster`). Uses `OnProcessStart` to delay spawners until the entity exists. |
| `generate_launch_script.sh` | `scripts/launch_<pkg>` | Workspace-level convenience launcher. |
| `robot_description_template.xacro` | — | Legacy fallback template referenced only by `render_template_xacro` (**dead code** — see [Cleanup Targets](#headline-observations--cleanup-targets)). |

### Conventions baked into the generator

- Robot keys are always `robot1`, `robot2`, … (set in `create-ros-pkg`).
- Joint naming pattern: `<key>_shoulder_pan_joint`, `_shoulder_lift_joint`, `_elbow_joint`, `_wrist_{1,2,3}_joint`.
- All generated packages depend on the in-tree `multi_ur_description` for `ur_macro.xacro` and per-model config YAMLs (`ur3`, `ur3e`, …, `ur30`).
- Hardware plugin is hardcoded to `ign_ros2_control/IgnitionSystem` — Ignition Fortress, not Gazebo classic.

---

## Collision Detection & Avoidance

Collision handling has two complementary halves: **avoidance** (planner-side, prevents bad trajectories) and **detection** (runtime, confirms the system actually catches contact when it happens). Both are wired into the generated package — no manual MoveIt Setup Assistant step required.

### Avoidance — handled at package-generation time

`scripts/helpers/generate_srdf.sh` writes `<pkg>/config/<pkg>.srdf` with three layers of rules:

| Layer | What it does |
| --- | --- |
| **Planning groups** | One MoveIt group per robot, defined as the chain `<key>_base_link_inertia` → `<key>_tool0`. Lets the planner reason about each arm independently. |
| **Self-collision disables** | Standard UR adjacency disables per arm (e.g. `shoulder_link` ↔ `upper_arm_link` as `Adjacent`, `base_link_inertia` ↔ `wrist_2_link` as `Never`). Without these, the planner rejects every pose as self-colliding. |
| **Inter-arm rules** | All arm-vs-arm collisions are **enabled by default** so the planner avoids inter-arm contact. The only inter-arm pairs explicitly disabled are: (a) base-to-base between consecutive robots (they're rigidly mounted), and (b) each robot's arm links against every *other* robot's base link (the bases are stationary obstacles, not collision risks for the moving links). |

Supporting MoveIt configs that fall out of the same generation step:

- `joint_limits.yaml` — velocity caps (π for arm joints, 2π for wrists), default scaling factor `0.1` so default-speed motion is conservative.
- `kinematics.yaml` — KDL solver per group.
- `pilz_cartesian_limits.yaml` — Cartesian velocity/acceleration limits for the Pilz planner.
- `moveit_controllers.yaml` + `controllers.yaml` — wire MoveIt to ros2_control trajectory controllers so plans actually execute on the simulated arms.

Net effect: as soon as `create-ros-pkg` finishes, the resulting package can plan collision-free trajectories in RViz/MoveIt for any number of UR arms without further setup.

### Detection — exercised at runtime by the integration suite

| Path | Role |
| --- | --- |
| `test/integration/collision_tests/simple_collisions_test.py` | The single live integration test. Drives the robots into overlapping joint configurations and verifies the collision-checking pipeline detects contact. |
| `scripts/integration-tests` | Sources `/opt/ros/humble/setup.bash` and the workspace overlay, then runs `pytest test/integration "$@"`. |
| `test/integration/collision_tests/utils/` | Shared helpers for the integration tests. |

The `test/other/` tree (`broken_tests/`, `tests_a/`, `tests_b/`, `tests_c/`) holds prior collision experiments — kept for reference, not run by CI or `pytest.ini`.

### Known gaps

- Only one collision test is currently active; the April 22 meeting flagged "make the test suite more comprehensive" as an open item.
- No regression test exercises `create-ros-pkg` itself across varying robot counts/poses to confirm the SRDF rules remain correct as the generator evolves.

---

## `src/` Packages

### Authored package skeletons

Originally derived from `ros_gz_project_template`; most are still placeholders.

| Package | State | Notes |
| --- | --- | --- |
| `multi_arm_lab_sim_application` | Empty | Only `CMakeLists.txt` + `package.xml`. |
| `multi_arm_lab_sim_bringup` | Active | Real launch files: `multiple_ur.launch.py`, `multiple_ur_with_gripper.launch.py`, `environment.launch.py`, `test.launch.py`, `xsarm.launch.py`, `old_mur.launch.py`, `parse_arm_config.py`. Config includes `arms.json`, controller/bridge YAMLs. |
| `multi_arm_lab_sim_description` | Lightly populated | Gazebo models: `lab/` (`.dae` mesh + textures), `thermoshaker/`, `ur3e/`. |
| `multi_arm_lab_sim_gazebo` | Stub plugins | C++ stubs `BasicSystem.cc`, `FullSystem.cc`; SDF worlds (`lab.sdf`, `thermoshaker.sdf`, `template.sdf`); `worlds_config.json`. |

### Vendored

| Package | Notes |
| --- | --- |
| `multi_ur_description` | Fork of `universal_robot` with extras: `config/ur3…ur30/`, `urdf/ur_macro.xacro`, `ur_with_robotiq_gripper.xacro`, and several experimental `arm1*.urdf` snapshots. |

### Reference / hand-built

| Package | Notes |
| --- | --- |
| `two_arm_moveit_config` | The hand-built two-UR-arm MoveIt package the generator is modeled after. Internal filenames carry the misspelling `mutli_arm.*`. |

### Generator outputs

| Package | Notes |
| --- | --- |
| `test_single` | Single-arm package produced by `create-ros-pkg`. |
| `test_single_29` | Same, different pose. |
| `test_double_6` | Two-arm output. |

`src/vendors/` exists but is empty.

---

## Tests

### Live

- `test/integration/collision_tests/simple_collisions_test.py` — the one canonical integration test, invoked via `scripts/integration-tests`.

### Graveyard — `test/other/`

| Directory | Notes |
| --- | --- |
| `broken_tests/` | Named for a reason — older planning/collision experiments left as reference. |
| `tests_a/` | Earlier collision suites; not wired into pytest. |
| `tests_b/` | Mixed Blender / world-creation / collision scripts. |
| `tests_c/` | Has its own `conftest.py`; not collected by the default `pytest.ini`. |

---

## Documentation

Sphinx-based, deployed via Read the Docs.

### `docs/guides/`

- `setup.md`
- `development.md`
- `loading_models.md`
- `creating_models.md`
- `create_new_sim_env.md`
- `customized_vendor_packages.md`
- `ros_pkg_generattion_scripts_arch.md` *(filename typo: "generattion")*
- `generated_ros_package_architecture.md`

### `docs/quick_info/`

- `software_dependencies_and_versions.md`

---

## Git State

- On branch `main`, up to date with `origin/main`.
- Only untracked path: `directed-studies/` (meeting notes — including this document).
- Recent commits (most recent first):

| SHA | Subject |
| --- | --- |
| `afc7e0e` | Add remaining documentation and small update to README |
| `ff99c8d` | Remove unused helper file for the ros package generation. |
| `d4cb2ab` | Automatically generate launch command scripts after package has been generated. |
| `9c98c82` | Fix multi-robot package generation |
| `90607be` | Create working script that generates a ROS package based on user inputs. |

**The generator pipeline is the active workstream** — recent commits all iterate on it.

---

## Headline Observations & Cleanup Targets

The pipeline works end-to-end but carries some accumulated cruft. Items to address (roughly highest-value first):

1. **`create-ros-pkg` overwrites helper output.** Lines 539–589 of `create-ros-pkg` write inline `package.xml` and `CMakeLists.txt` blocks that clobber the richer files produced by `generate_package_xml.sh` / `generate_cmakelists.sh`. The inline `package.xml` drops most MoveIt `exec_depend`s and changes the maintainer string from Roman to `user@example.com`. Either delete the inline writers or delete the helpers — one of them is wasted work.

2. **Duplicate helper invocations.** `generate_controllers.sh` and `generate_rviz_config.sh` are each called twice in `main()` — once in the main fan-out and again at the bottom. Redundant; harmless but confusing.

3. **Dead code: `render_template_xacro`.** Defined at `create-ros-pkg:347–405` but never called. It also uses GNU `sed -i -E` syntax that would fail on macOS BSD `sed` if revived. Either wire it up behind a flag or delete it.

4. **Implicit cross-package dependency.** `generate_sensors_3d.sh` hard-depends on `src/two_arm_moveit_config/config/sensors_3d.yaml` existing. If that reference package is renamed or removed, every new package silently gets the empty `sensors: []` fallback.

5. **Errors are silenced.** `expand_xacro_to_urdf` and `generate_static_robot_description.sh` redirect xacro stderr to `/dev/null`. Real failures during package generation will be invisible to the user. At minimum, propagate non-zero exit codes.

6. **`mutli_arm` typo propagates.** `two_arm_moveit_config/` carries the misspelling `mutli_arm.*` through filenames, SRDF, and XACRO. `generate_robot_description_wrapper.sh` mirrors it in a comment. Cosmetic, but breaks `grep` and reads as a bug.

7. **No tests for the generator itself.** The `test/` tree only covers runtime collision behavior. There's no smoke test that runs `create-ros-pkg` non-interactively across a small grid of robot counts/models to catch regressions in helper output.

### Next steps per the May 15 2026 meeting

1. Thoroughly document the current state of the project. *(← this document is part of that effort.)*
2. Survey what software / packages other self-driving labs are using (Matter, Berlinguette, +4 more in Discord).
3. Produce a new plan with concrete timeline and tangible tasks.
4. Update `setup.sh` and overall documentation.
5. Stress-test the bringup scripts with varied counts/positions/placements.
6. Place objects in the brought-up environment and construct an environment "by hand"; figure out an efficient workflow for that.
7. Add compatibility for non-UR robots.
