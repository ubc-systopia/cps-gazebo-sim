# Arm Control API — Design & Build Log

> **Date:** 2026-05-26
> **Author:** Roman (design via pairing session)
> **Scope:** A scriptable Python API for commanding the UR arms in a generated
> MoveIt + Ignition Gazebo package, plus the first single-file test script that
> exercises it.

---

## 1. Goal

End goal: a **library of functions (an "API")** for controlling the robot arms,
so workflow scripts read as a sequence of high-level calls rather than raw ROS
plumbing. Modelled after how a sibling repo drives a Niryo NED2 with
[`pyniryo`](https://docs.niryo.com/) — its core primitive is
`ned2.move_pose([x, y, z, roll, pitch, yaw])`, with thin wrappers and
higher-level pick/place orchestration layered on top.

**This iteration:** do it all in **one test script** that can be executed while
Gazebo and MoveIt are running (i.e. while `ros2 launch <pkg> demo.launch.py` from
a generated package is up). The script is written so its internals lift straight
out into a library later with no rewrite.

---

## 2. Target system recap

The generated package (e.g. `src/diff_robots/`) brings up, via its
`launch/demo.launch.py`:

- **Ignition Gazebo** with the multi-arm robot spawned from `/robot_description`.
- **`move_group`** (MoveIt 2) with one planning group per arm, taken from the
  SRDF: `robot1`, `robot2`, … each a `base_link_inertia → tool0` chain.
- **`ros2_control`** controllers: one `JointTrajectoryController` per arm
  (`robotN_joint_trajectory_controller`) plus a `joint_state_broadcaster`.
- **RViz** with a MotionPlanning panel per arm.

`diff_robots` specifically: `robot1` = UR5 at world `(0,0,0)`, `robot2` = UR3 at
world `(1,1,0)`. Tip link `robotN_tool0`; base link `robotN_base_link`.

---

## 3. Design decisions (and why)

| Decision | Choice | Rationale |
| --- | --- | --- |
| **Goal type** | Full 6-DOF Cartesian **pose** (`[x, y, z, roll, pitch, yaw]`) | User wants to "enter positions of the end of the arm." Mirrors pyniryo's `move_pose([...])`. The one primitive direct controllers can't do — needs IK + planning. |
| **Control layer** | **MoveIt** (collision-aware planning + IK) | Pose/Cartesian goals require it. RViz MotionPlanning panel is exactly this, scripted. |
| **MoveIt access** | **`MoveGroup` action interface** over `rclpy` | `moveit_py` is **not installed** in this environment (only the C++ `moveit_ros_planning_interface`). The action interface talks to the `move_group` already launched by `demo.launch.py`, needs no extra install, and uses stock message packages (`moveit_msgs`, `geometry_msgs`, `shape_msgs`). |
| **Reference frame** | **Per-arm base frame** (`robotN_base_link`) | Mirrors pyniryo (base-relative poses). Makes the same pose list portable between arms. Costs nothing extra: the goal's `frame_id` is set to the arm's base link and MoveIt/TF transforms it into the planning frame. |
| **API shape** | `ArmFleet` session + per-arm `Arm` handle | `fleet.arm("robot1")` returns a handle whose `.move_pose(...)` mirrors pyniryo's `ned2.move_pose(...)`. One `ArmFleet` owns the rclpy node + action client; each `Arm` carries its group/tip/base names. This pair *is* the future library. |
| **Blocking** | `move_pose` / `home` **block until execution completes** | The action result tells us success/failure — no fixed `sleep()` guessing (pyniryo needed `time.sleep(delay)` because its TCP command returned early). |
| **Pick / place** | **Skipped for now** | The URDF has **no gripper / end-effector** (SRDF chain ends bare at `tool0`). `open_gripper`/`close_gripper` and full pick-place need a gripper added to the description + a gripper controller first. The waypoint pattern (approach → safe height → target) can be layered on later as motion-only. |

### Why not `package.xml` surgery?

An earlier concern was that the generated `package.xml` is the stripped-down
inline version with no MoveIt dependencies, which would block a `moveit_py`
script. Because we pivoted to the **action interface running standalone against
the live `move_group`**, the script depends only on system message packages —
so no `package.xml` change is required to run this test.

---

## 4. API surface (this iteration)

```python
fleet  = ArmFleet()                 # rclpy node + MoveGroup action client
robot1 = fleet.arm("robot1")        # per-arm handle (≈ pyniryo's `ned2`)

robot1.move_pose([0.4, 0.1, 0.4, 0.0, 1.5708, 0.0])   # base-frame pose on tool0
robot1.move_joints([0, -1.5708, 0, -1.5708, 0, 0])    # joint-space (deterministic)
robot1.home()                                          # horizontal (all zeros)
robot1.upright()                                       # lifted vertical, floor-clear

# Move multiple arms at the SAME time (joint-space goals):
fleet.move_all({"robot1": UPRIGHT_JOINTS, "robot2": UPRIGHT_JOINTS})
```

**Simultaneous multi-arm motion (`move_all`).** MoveIt's `/move_action` server is
**single-goal** — firing two plan-and-execute goals at it would make the second
preempt the first, so it can't drive two arms concurrently. Instead `move_all`:
1. **plans** each arm separately via `/move_action` with `plan_only=True` (fast,
   sequential) to obtain each arm's `RobotTrajectory`;
2. **executes** every trajectory at once by sending each to its *own*
   `robotN_joint_trajectory_controller/follow_joint_trajectory`
   (`control_msgs/FollowJointTrajectory`) action — these are independent
   per-controller servers, so the arms move in parallel.

It sends all trajectory goals, then waits on all results; planning failure for
any arm aborts before anything moves. Joint-space targets only (deterministic,
floor-safe).

**Coordinated multi-arm motion (`move_coordinated`).** `move_all` plans each arm
against a *frozen snapshot* of the others, so it isn't collision-safe while both
move. For arms that share workspace, `move_coordinated` instead plans all arms in
a single **composite SRDF group** (`all_arms`) — one combined plan that's
collision-free *while every arm moves* — and `move_group` auto-splits the
resulting trajectory across each arm's controller for simultaneous execution.

Group design (per the digital-twin / SDL goal): **per-arm groups stay the default
working unit** (cheap planning, Cartesian/IK goals, modular, scalable); the
composite `all_arms` group is *added* for the cases that truly need synchronized
motion. The generator (`generate_srdf.sh`) now emits per-arm groups **plus** an
`all_arms` composite whenever ≥2 robots are configured. Adding/using the
composite group requires a `colcon build` + relaunch (SRDF loads at move_group
startup). Joint-space only — composite groups have no single IK chain.

**Joint-space vs Cartesian for floor clearance:** at all-zeros a UR arm is
stretched straight out *horizontally*, so a Cartesian `move_pose` can let IK pick
a solution/path that swings the arm down **through the floor**. `move_joints`
specifies joint angles directly — deterministic, no IK — so named poses like
`upright` (`shoulder_lift = -π/2` lifts the upper arm vertical) are guaranteed
floor-clear. `home`/`upright` are thin wrappers over `move_joints`.

- `move_pose([x, y, z, roll, pitch, yaw])` — builds a `Constraints` message with
  a `PositionConstraint` (small sphere region around the target point on
  `robotN_tool0`, in `robotN_base_link`) and an `OrientationConstraint` (RPY →
  quaternion), sends a `MoveGroup` goal with `plan_only=False` so `move_group`
  plans **and** executes via the controllers. Returns `True`/`False`.
- `home()` — joint-space goal driving all six joints to `0.0` (matches
  `initial_positions.yaml` in the example). Guaranteed reachable; good smoke test.

### Conventions baked in

- Planning group: `robotN`
- Tip link (goal target): `robotN_tool0`
- Goal frame: `robotN_base_link`
- Joints: `robotN_{shoulder_pan,shoulder_lift,elbow,wrist_1,wrist_2,wrist_3}_joint`
- `use_sim_time=True` (Gazebo owns the clock)

---

## 5. How to run

In one terminal, bring the package up:

```bash
cd <workspace_root>
./scripts/launch_diff_robots          # colcon build + source + ros2 launch demo
```

In a second terminal, once `move_group` and Gazebo are up:

```bash
cd <workspace_root>
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/diff_robots/scripts/workflow_demo.py
```

The demo imports the `multi_arm_control` library, so that package must be built
and sourced (it is, via `install/setup.bash`); the demo script itself needs no
rebuild. For ad-hoc Cartesian targets there's also
`python3 src/diff_robots/scripts/move_pose_cli.py <arm> <x y z r p y>`.

> Note: the original single-file `move_arm_test.py` (referenced in §1) was
> superseded by the library + `workflow_demo.py` — see §5b.

---

## 5b. Architecture: central library + thin per-package demo

The control logic now lives in a **central reusable package**, and generated
packages carry only configuration + a thin task script:

```
src/multi_arm_control/                 # the API (behavior / verbs)
  multi_arm_control/
    fleet.py         # ArmFleet, Arm, rpy_to_quat
    conventions.py   # naming contract (joint/link/group/controller patterns)
    __init__.py      # exports ArmFleet, Arm, HOME_JOINTS, UPRIGHT_JOINTS, ...
  package.xml/setup.py   # ament_python

src/<generated_pkg>/                   # configuration (nouns) + choreography
  config/ launch/ urdf/  ...           # as before
  package.xml            # + <exec_depend>multi_arm_control</exec_depend>
  scripts/workflow_demo.py             # THIN: imports the library, ~30 lines
```

**Data-driven, not per-package.** `ArmFleet(package_name)` reads that package's
installed `config/robots.json` to discover arm keys, then derives every ROS name
from `conventions.py`. So one library drives any generated package with zero
per-package API code; the demo is package-agnostic (iterates `fleet.arm_names`).

**Conventions contract.** `conventions.py` is the single source of truth for the
naming the generator bakes in (`robotN_*_joint`, `robotN_tool0`,
`robotN_base_link`, `robotN_joint_trajectory_controller`, groups `robotN` +
`all_arms`). Generator and library must agree; a naming change updates both.

**Generator integration.** `create-ros-pkg` now calls
`helpers/generate_workflow_demo.sh` (emits the thin `scripts/workflow_demo.py`
pre-filled with the package name) and adds the `multi_arm_control` exec_depend to
the generated `package.xml`. `diff_robots` was converted by hand as the reference
(its monolithic `move_arm_test.py` was replaced by `scripts/workflow_demo.py`).

Decisions: **new dedicated package** (single responsibility, reusable) over
folding into `multi_arm_lab_sim_application`; generated script is a **runnable
demo** (doubles as a wiring smoke test) rather than an empty stub.

## 5c. Cartesian multi-arm motion (`move_poses`) and the test demo

`move_all`/`move_coordinated` are joint-space only; there is no combined-group IK
(composite groups have no single chain). To send **multiple arms to Cartesian
targets at once**, `move_poses(targets)` is the IK analogue of `move_all`: it
plans each arm's pose on its **own** group (so IK runs per arm), then executes
all trajectories in parallel via each arm's controller. Independent plans (frozen
snapshot) -> not collision-coordinated, same caveat as `move_all`.

`Arm.pose_constraints(pose)` was extracted from `move_pose` so both reuse it.

The generated `workflow_demo.py` is now a small **test runner** with a PASS/FAIL
summary, covering both control modes x both timings:

| Part | What it exercises |
| --- | --- |
| 1. smoke | `move_joints` home on every arm (pipeline alive) |
| 2a. joints simultaneous | `move_coordinated` (collision-checked together) |
| 2b. joints sequential | per-arm `move_joints` up/down |
| 3a. IK sequential | per-arm `move_pose` to `DEMO_POSE`, then home |
| 3b. IK simultaneous | `move_poses` (parallel per-arm IK), then home |

`DEMO_POSE` is a conservative base-frame target; per-package tuning (via
`move_pose_cli.py`) may be needed if an arm reports a planning failure. The
generator template and `diff_robots`' copy are byte-identical.

### In-sim results & the simultaneous-IK fix

First full in-sim run: parts 1, 2a, 2b, and **3a (IK sequential) PASSED** — this
**confirms `move_pose` end-to-end** (both arms reached the Cartesian target).
Part **3b failed**: the plan-only path (`_plan`, used by `move_poses`/`move_all`)
returned `INVALID_MOTION_PLAN` *instantly* for a pose that plans fine via
plan+execute — move_group's plan-only path rejects the Cartesian goal pre-planning.

Fix (Option 2): **`move_coordinated_poses(targets)`** resolves each pose to joint
angles via the `/compute_ik` service (`compute_ik`, `avoid_collisions=True`), then
routes through `move_coordinated`. This uses only proven paths and yields a single
**collision-coordinated** trajectory (better than `move_poses`' independent
parallel plans). Demo 3b now uses it. Caveat: `move_all`/`move_poses` still rely
on the suspect plan-only path and are unverified — prefer the coordinated methods.
Error logs now decode MoveItErrorCodes to names (e.g. `-2 (INVALID_MOTION_PLAN)`).

### IK solver: KDL -> pick_ik

Root cause of the "pretzel" folding *and* 3b's instant `INVALID_MOTION_PLAN`: the
generated `kinematics.yaml` used `KDLKinematicsPlugin` — a local numerical solver
that returns wound, near-joint-limit configurations (UR wrists allow ±2pi) and
accumulates winding across calls (each seeds from the previous state). `compute_ik`
returned such a config and the combined-group goal was rejected pre-planning.

Fix: switched to **`pick_ik/PickIkPlugin`** (PickNik's global, joint-limit-aware
solver; `apt install ros-humble-pick-ik`). Updated `generate_kinematics.sh` (now
emits pick_ik in global mode, timeout 0.05; also fixed a latent append-not-truncate
bug) and `diff_robots`' `kinematics.yaml`; added `<exec_depend>pick_ik</exec_depend>`
to the generator + `diff_robots`. Requires rebuild + relaunch for move_group to
load the new solver.

**Result (2026-05-27):** after the switch, the full demo passes in sim — all five
tests including 3b (IK simultaneous coordinated), no pretzel folding. The complete
API (joint + Cartesian, single + multi-arm, sequential + coordinated) is verified
end-to-end. Remaining: planning is slow (~12-24 s/move) — an OMPL planning-time
tuning task, not a correctness issue.

## 6. Known limitations / future work

- **Reachability ≠ portability.** Base-frame coords make a pose list reusable
  across arms, but a pose easy for `robot1` may be physically unreachable for
  `robot2`. `move_pose` surfaces this as a planning failure (returns `False`).
- **No gripper / pick-place yet.** Needs an end-effector in the URDF + controller.
- **Library extraction — DONE.** `ArmFleet`/`Arm` now live in the dedicated
  `multi_arm_control` package; generated packages carry only a thin
  `workflow_demo.py` caller (see §5b).
- **Example pose values are illustrative** — tune to your workspace.
- **`move_all` / `move_poses` unverified** — they ride the plan-only path that
  fails for Cartesian goals; prefer the coordinated methods. Diagnose or retire.
- **Planning is slow (~12–24 s/move)** — OMPL planning-time tuning is a future
  perf task (e.g. fewer `num_planning_attempts`, a faster planner).

---

## 7. Build log (actions taken)

1. Verified `moveit_py` is **not** installed; confirmed `moveit_msgs`,
   `geometry_msgs`, `shape_msgs` are available → chose the `MoveGroup` action
   interface. (`tf_transformations` also absent → inline RPY→quaternion helper.)
2. Wrote this design doc.
3. Wrote the test script at `src/diff_robots/scripts/move_arm_test.py` with the
   `ArmFleet` / `Arm` structure described above.
4. Validated the script compiles (`python3 -m py_compile`).
5. **Verified end-to-end in sim** against a running `diff_robots` launch:
   `home` and `move_pose` both reached their goals and `robot1` (UR5) physically
   moved in Gazebo; controllers confirmed `active` via `ros2 control
   list_controllers`. Caveat: the **first** planning request after launch is slow
   (move_group lazily loads OMPL planner plugins), and the script gave no interim
   feedback, so it initially looked hung.
6. Responsiveness tuning based on that run:
   - default `vel_scale`/`acc_scale` raised `0.1 → 0.3` (~3× faster execution);
   - orientation tolerance loosened `0.01 → 0.05` rad (faster, more reliable
     planning);
   - added a "goal accepted - planning + executing..." log line and a
     `feedback_callback` that prints move_group's state transitions, so progress
     is visible between send and result.
7. Added joint-space named poses (`home`/`upright`) over a `move_joints`
   primitive, since Cartesian goals let IK swing the arm through the floor.
8. Added `move_all` for **simultaneous** multi-arm motion (plan-each via
   `/move_action` plan-only, then execute all on per-arm
   `FollowJointTrajectory` controllers in parallel). `main()` now lifts both
   `robot1` and `robot2` upright together, then home. Verified the module
   compiles and the constraint/controller wiring is correct; awaiting a live
   in-sim run to confirm visual simultaneity.
9. Added an `all_arms` composite group (per-arm groups kept as default) to both
   `diff_robots.srdf` and the generator `generate_srdf.sh` (emitted for ≥2
   robots), plus `move_coordinated()` for collision-checked-together motion.
   `main()` now uses `move_coordinated`. Verified: script compiles, merged goal
   has 12 joint constraints, generator emits the group for N robots, and
   `diff_robots` rebuilt with the new SRDF installed. **Needs a relaunch** for
   move_group to load the new group before an in-sim run.
10. **Extracted the API into the central `multi_arm_control` package**
    (`fleet.py`/`conventions.py`/`__init__.py`, data-driven via `robots.json`);
    converted `diff_robots` to a thin `scripts/workflow_demo.py` importing it
    (removed `move_arm_test.py`); wired `create-ros-pkg` to emit the demo +
    `multi_arm_control` exec_depend. Added `move_pose_cli.py` and fleet-level
    `move_pose`/`move_joints` wrappers. (See §5b.)
11. First full in-sim run: smoke/2a/2b/**3a PASS — `move_pose` confirmed
    end-to-end**. 3b (`move_poses`, plan-only path) failed instantly with
    `INVALID_MOTION_PLAN`.
12. Added `compute_ik` (`/compute_ik` service) + `move_coordinated_poses`
    (pose→joints→`move_coordinated`) for collision-coordinated simultaneous
    Cartesian motion; demo 3b switched to it. Error logs now decode codes to
    names. (See §5c.)
13. Diagnosed the pretzel folding *and* 3b's instant failure as **KDL IK quality**;
    switched `kinematics.yaml` to **`pick_ik`** (installed `ros-humble-pick-ik`),
    updated `generate_kinematics.sh` (+ append-bug fix), added `pick_ik`
    exec_depend. Re-ran in sim: **all five tests PASS, no folding — full API
    verified end-to-end.** (See §5c "IK solver".)
