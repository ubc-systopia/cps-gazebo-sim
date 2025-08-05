import pytest
import rclpy
from rclpy.node import Node
import time
import sys
print(sys.path)

# Direct ROS2 imports for MoveIt2
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest,
    RobotState,
    Constraints,
    JointConstraint,
    WorkspaceParameters
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

import numpy as np
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_testing.actions import ReadyToTest
from launch_testing.launch_test_decorator import launch_test

@launch_test
def generate_test_description():
    ld = LaunchDescription()

    # Include the demo.launch.py file
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arm1g_transmission_moveit_config'),
                'launch',
                'demo.launch.py'
            )
        )
    )

    ld.add_action(demo_launch)
    ld.add_action(ReadyToTest())

    return ld

class TestMovement(Node):
    def __init__(self):
        super().__init__('test_movement')

        # Initialize MoveIt2 service clients
        self.get_logger().info('Initializing MoveIt2 service clients...')

        # Motion planning service
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')

        # Action client for trajectory execution
        self.execute_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/ur3e_arm_controller/arm1_joint_trajectory_controller'  # Adjust controller name as needed
        )

        # Joint state subscriber to get current robot state
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_joint_state = None

        # Joint names for the arm
        self.arm1_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.plan_client.wait_for_service(timeout_sec=10.0)
        self.execute_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('Services ready!')

    def joint_state_callback(self, msg):
        """Callback to store current joint state"""
        self.current_joint_state = msg

    def create_joint_constraints(self, joint_names, joint_positions, tolerance=0.01):
        """Create joint constraints for motion planning"""
        constraints = Constraints()

        for i, (name, position) in enumerate(zip(joint_names, joint_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = tolerance
            joint_constraint.tolerance_below = tolerance
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        return constraints

    def create_robot_state(self, joint_names, joint_positions):
        """Create a RobotState message"""
        robot_state = RobotState()

        # Create joint state
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = joint_names
        joint_state.position = [float(pos) for pos in joint_positions]

        robot_state.joint_state = joint_state
        return robot_state

    def plan_to_joint_positions(self, joint_names, joint_positions):
        """Plan motion to joint positions using MoveIt2 service"""
        if not self.current_joint_state:
            self.get_logger().error('No current joint state available')
            return None

        # Create motion plan request
        request = GetMotionPlan.Request()

        # Set up the request
        req = MotionPlanRequest()
        req.workspace_parameters = WorkspaceParameters()
        req.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        req.workspace_parameters.header.frame_id = "base_link"  # Adjust frame as needed
        req.workspace_parameters.min_corner.x = -2.0
        req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -2.0
        req.workspace_parameters.max_corner.x = 2.0
        req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 2.0

        # Set start state (current state)
        req.start_state = RobotState()
        req.start_state.joint_state = self.current_joint_state

        # Set goal constraints
        req.goal_constraints = [self.create_joint_constraints(joint_names, joint_positions)]

        # Set group name
        req.group_name = "ur3e_arm"

        # Set planner
        req.planner_id = "RRTConnect"  # or "OMPL" or other available planners
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1

        request.motion_plan_request = req

        # Call the service
        self.get_logger().info('Calling motion planning service...')
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result() is not None:
            response = future.result()
            if response.motion_plan_response.error_code.val == 1:  # SUCCESS
                self.get_logger().info('Planning successful!')
                return response.motion_plan_response.trajectory
            else:
                self.get_logger().error(f'Planning failed with error code: {response.motion_plan_response.error_code.val}')
                return None
        else:
            self.get_logger().error('Service call failed')
            return None

    def execute_trajectory(self, trajectory):
        """Execute a planned trajectory"""
        if not trajectory or not trajectory.joint_trajectory.points:
            self.get_logger().error('Invalid trajectory')
            return False

        # Create action goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory

        self.get_logger().info('Sending trajectory for execution...')
        future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory execution rejected')
            return False

        # Wait for execution to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.result.error_code == 0:  # SUCCESS
            self.get_logger().info('Trajectory execution successful!')
            return True
        else:
            self.get_logger().error(f'Trajectory execution failed with error: {result.result.error_code}')
            return False

    def move_arm_to_joint_positions(self, joint_names, joint_positions):
        """Plan and execute motion to joint positions"""
        try:
            # Plan the motion
            trajectory = self.plan_to_joint_positions(joint_names, joint_positions)
            if trajectory is None:
                return False

            # Execute the trajectory
            return self.execute_trajectory(trajectory)

        except Exception as e:
            self.get_logger().error(f'Error in move_arm_to_joint_positions: {e}')
            return False

    def run_demo_sequence(self):
        """Run a demo sequence moving the arm"""
        self.get_logger().info('Starting simple arm demo sequence...')

        # Wait for current joint state
        while self.current_joint_state is None:
            self.get_logger().info('Waiting for joint state...')
            rclpy.spin_once(self, timeout_sec=1.0)

        # Define some demo positions (in radians)
        home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        arm1_pos1 = [0.5, -1.2, 0.5, -1.0, 0.0, 0.0]
        arm1_pos2 = [0.2, -1.0, 0.3, -0.8, 0.1, 0.0]

        try:
            # Move to home position
            self.get_logger().info('Moving arm to home position...')
            success1 = self.move_arm_to_joint_positions(self.arm1_joints, home_position)
            self.get_logger().info(f'Home move: {"SUCCESS" if success1 else "FAILED"}')

            time.sleep(2)

            # Move to position 1
            self.get_logger().info('Moving to position 1...')
            success1 = self.move_arm_to_joint_positions(self.arm1_joints, arm1_pos1)
            self.get_logger().info(f'Pos1 move: {"SUCCESS" if success1 else "FAILED"}')

            time.sleep(2)

            # Move to position 2
            self.get_logger().info('Moving to position 2...')
            success1 = self.move_arm_to_joint_positions(self.arm1_joints, arm1_pos2)
            self.get_logger().info(f'Pos2 move: {"SUCCESS" if success1 else "FAILED"}')

            time.sleep(2)

            # Return to home
            self.get_logger().info('Returning to home position...')
            success1 = self.move_arm_to_joint_positions(self.arm1_joints, home_position)
            self.get_logger().info(f'Return home: {"SUCCESS" if success1 else "FAILED"}')

            self.get_logger().info('Demo sequence completed!')

        except Exception as e:
            self.get_logger().error(f'Demo sequence failed: {str(e)}')

@pytest.mark.launch_test
def test_movement():
    rclpy.init()

    controller = TestMovement()

    try:
        # Wait a bit for everything to initialize
        time.sleep(5)
        controller.run_demo_sequence()

        # Keep the node alive for a bit
        time.sleep(2)

    except KeyboardInterrupt:
        controller.get_logger().info('Demo interrupted by user')
    except Exception as e:
        controller.get_logger().error(f'Demo failed: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()