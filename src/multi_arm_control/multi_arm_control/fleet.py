"""ArmFleet / Arm - the reusable control API.

An ``ArmFleet`` owns one rclpy node and a ``MoveGroup`` action client, and is
configured purely by a generated package's name: it reads that package's
``config/robots.json`` to discover the arms, then derives all ROS names from
:mod:`multi_arm_control.conventions`. So one library drives any package the
generator produces, with no per-package code.

Motion goes through the already-running ``move_group`` via the ``MoveGroup``
action interface (no moveit_py required).
"""

import json
import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory

from . import conventions as conv


def rpy_to_quat(roll, pitch, yaw):
    """Convert roll/pitch/yaw (radians) to an (x, y, z, w) quaternion.

    Inline to avoid a dependency on tf_transformations.
    """
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def _load_arm_names(package_name):
    """Read arm keys from a generated package's config/robots.json."""
    share = get_package_share_directory(package_name)
    with open(os.path.join(share, "config", "robots.json")) as f:
        data = json.load(f)
    return [a["key"] for a in data.get("robot_arms", []) if a.get("key")]


class Arm:
    """A handle to one planning group. Mirrors pyniryo's single-robot object."""

    def __init__(self, fleet, name):
        self.fleet = fleet
        self.name = name
        self.group = conv.planning_group(name)
        self.tip_link = conv.tip_link(name)
        self.base_frame = conv.base_frame(name)

    # --- core primitive --------------------------------------------------
    def move_pose(self, pose, vel_scale=0.3, acc_scale=0.3):
        """Move the end of the arm (tool0) to a 6-DOF pose in the arm's base frame.

        pose = [x, y, z, roll, pitch, yaw]  (metres, radians).
        Returns True on success, False on planning/execution failure.
        """
        x, y, z, roll, pitch, yaw = pose
        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)

        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.base_frame
        pos_c.link_name = self.tip_link
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [0.01]
        target = Pose()
        target.position.x = float(x)
        target.position.y = float(y)
        target.position.z = float(z)
        target.orientation.w = 1.0
        pos_c.constraint_region.primitives = [region]
        pos_c.constraint_region.primitive_poses = [target]
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.base_frame
        ori_c.link_name = self.tip_link
        ori_c.orientation.x = qx
        ori_c.orientation.y = qy
        ori_c.orientation.z = qz
        ori_c.orientation.w = qw
        ori_c.absolute_x_axis_tolerance = 0.05
        ori_c.absolute_y_axis_tolerance = 0.05
        ori_c.absolute_z_axis_tolerance = 0.05
        ori_c.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pos_c]
        constraints.orientation_constraints = [ori_c]

        return self.fleet._plan_and_execute(
            self.group, constraints, vel_scale, acc_scale,
            desc=f"{self.name} -> pose {pose}",
        )

    def joint_constraints(self, positions):
        """Build a joint-space goal Constraints msg for this arm.

        positions = 6 values for [pan, lift, elbow, wrist_1, wrist_2, wrist_3] (rad).
        """
        constraints = Constraints()
        for joint_name, val in zip(conv.joint_names(self.name), positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(val)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    # --- convenience -----------------------------------------------------
    def move_joints(self, positions, vel_scale=0.3, acc_scale=0.3):
        """Joint-space move. Deterministic (no IK), so it won't dip through the
        floor the way a Cartesian goal can."""
        return self.fleet._plan_and_execute(
            self.group, self.joint_constraints(positions), vel_scale, acc_scale,
            desc=f"{self.name} -> joints {list(positions)}",
        )

    def home(self, **kw):
        """All-zeros 'horizontal' pose (matches initial_positions.yaml)."""
        return self.move_joints(conv.HOME_JOINTS, **kw)

    def upright(self, **kw):
        """Upright pose: upper arm lifted to vertical, clear of the floor."""
        return self.move_joints(conv.UPRIGHT_JOINTS, **kw)


class ArmFleet(Node):
    """Owns the rclpy node + action clients; hands out Arm handles.

    package_name selects which generated package's robots.json to read.
    """

    def __init__(self, package_name, action_name="move_action"):
        super().__init__("multi_arm_control")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.package_name = package_name
        self.arm_names = _load_arm_names(package_name)
        self._client = ActionClient(self, MoveGroup, action_name)
        self._arms = {}
        self._fjt = {}  # per-arm FollowJointTrajectory controller action clients
        self.get_logger().info(
            f"loaded {len(self.arm_names)} arm(s) from "
            f"{package_name}/config/robots.json: {self.arm_names}")

    def arm(self, name):
        if name not in self.arm_names:
            raise KeyError(f"{name!r} not in {self.package_name} "
                           f"(known arms: {self.arm_names})")
        if name not in self._arms:
            self._arms[name] = Arm(self, name)
        return self._arms[name]

    def _fjt_client(self, name):
        """Action client for an arm's joint_trajectory_controller (independent
        per-arm servers - this is what lets us execute arms in parallel)."""
        if name not in self._fjt:
            self._fjt[name] = ActionClient(
                self, FollowJointTrajectory, conv.controller_action(name))
        return self._fjt[name]

    def _plan_and_execute(self, group, constraints, vel_scale, acc_scale, desc):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("move_group action server not available "
                                     "(is the package launched?)")
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.goal_constraints = [constraints]
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = vel_scale
        goal.request.max_acceleration_scaling_factor = acc_scale
        goal.planning_options.plan_only = False  # plan AND execute

        self.get_logger().info(f"[{desc}] sending goal...")

        def on_feedback(msg):
            self.get_logger().info(f"[{desc}]   ...{msg.feedback.state}")

        send_future = self._client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"[{desc}] goal rejected")
            return False

        self.get_logger().info(f"[{desc}] accepted - planning + executing "
                               "(first plan after launch is the slowest)...")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"[{desc}] SUCCESS")
            return True
        self.get_logger().error(f"[{desc}] FAILED (MoveItErrorCode {code})")
        return False

    # --- simultaneous multi-arm motion -----------------------------------
    def move_all(self, targets, vel_scale=0.3, acc_scale=0.3):
        """Move several arms to joint-space goals at the SAME time.

        targets = {"robot1": [6 joint vals], "robot2": [6 joint vals], ...}

        move_group's action server is single-goal, so we PLAN each arm
        separately (fast, sequential) and then EXECUTE every trajectory at once
        on each arm's own controller (independent servers -> true parallelism).
        NOTE: each plan ignores the others' motion (frozen snapshot), so this is
        NOT collision-safe while arms move - use move_coordinated for that.
        """
        plans = {}
        for name, positions in targets.items():
            arm = self.arm(name)
            traj = self._plan(arm.group, arm.joint_constraints(positions),
                              vel_scale, acc_scale, desc=name)
            if traj is None:
                self.get_logger().error(f"[move_all] planning failed for {name}; "
                                        "aborting (no arm moved)")
                return False
            plans[name] = traj
        return self._execute_all(plans)

    def _plan(self, group, constraints, vel_scale, acc_scale, desc):
        """Plan only (no execution). Returns a JointTrajectory or None."""
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("move_group action server not available")
            return None

        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.goal_constraints = [constraints]
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = vel_scale
        goal.request.max_acceleration_scaling_factor = acc_scale
        goal.planning_options.plan_only = True  # plan, DON'T execute

        self.get_logger().info(f"[move_all] planning {desc}...")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"[move_all] plan goal rejected for {desc}")
            return None

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"[move_all] planning {desc} FAILED "
                                    f"(MoveItErrorCode {result.error_code.val})")
            return None
        return result.planned_trajectory.joint_trajectory

    def _execute_all(self, plans):
        """Send every planned trajectory to its controller, then await all."""
        result_futures = {}
        for name, traj in plans.items():
            client = self._fjt_client(name)
            if not client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f"[move_all] controller for {name} "
                                        "unavailable")
                return False
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            send_future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f"[move_all] {name} trajectory rejected")
                return False
            result_futures[name] = handle.get_result_async()

        self.get_logger().info(f"[move_all] executing {len(result_futures)} arms "
                               "simultaneously...")
        ok = True
        for name, rf in result_futures.items():
            rclpy.spin_until_future_complete(self, rf)
            res = rf.result().result
            if res.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().info(f"[move_all] {name} done")
            else:
                self.get_logger().error(f"[move_all] {name} execution FAILED "
                                        f"({res.error_code}: {res.error_string})")
                ok = False
        return ok

    # --- coordinated multi-arm motion (collision-checked together) -------
    def move_coordinated(self, targets, group=conv.COMBINED_GROUP,
                         vel_scale=0.3, acc_scale=0.3):
        """Move several arms as ONE coordinated, collision-checked-together motion.

        targets = {"robot1": [6 joint vals], "robot2": [6 joint vals], ...}

        Plans all arms in a single combined group so the motion is collision-free
        *while every arm moves*; move_group then auto-splits the trajectory across
        each arm's controller for simultaneous execution. Requires the composite
        group (default "all_arms") in the SRDF -> rebuild + relaunch after adding.
        Joint-space only (composite groups have no single IK chain).
        """
        combined = Constraints()
        for name, positions in targets.items():
            combined.joint_constraints += \
                self.arm(name).joint_constraints(positions).joint_constraints
        return self._plan_and_execute(group, combined, vel_scale, acc_scale,
                                      desc=group)
