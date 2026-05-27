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
python3 src/diff_robots/scripts/move_arm_test.py
```

No rebuild needed — the script runs directly with `python3` and connects to the
running `move_group`.

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

## 6. Known limitations / future work

- **Reachability ≠ portability.** Base-frame coords make a pose list reusable
  across arms, but a pose easy for `robot1` may be physically unreachable for
  `robot2`. `move_pose` surfaces this as a planning failure (returns `False`).
- **No gripper / pick-place yet.** Needs an end-effector in the URDF + controller.
- **Library extraction.** `ArmFleet` / `Arm` move into their own module
  (candidate home: `multi_arm_lab_sim_application`); `main()` becomes the caller.
- **Example pose values are illustrative** — tune to your workspace.

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
