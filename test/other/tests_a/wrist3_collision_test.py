#!/usr/bin/env python3
"""
Collision detection test for wrist3 joints of both UR3e robot arms.
This test uses ROS2 services directly without MoveIt Python bindings.
"""

import pytest
import rclpy
from rclpy.node import Node
import time
import numpy as np
from threading import Event

# ROS2 imports for collision detection and robot control
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState, PlanningScene
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

class WristCollisionTester(Node):
    def __init__(self):
        super().__init__('wrist_collision_tester')
        
        self.get_logger().info('Initializing wrist collision tester...')
        
        # Robot joint names for both arms
        self.arm1_joints = [
            'arm1_shoulder_pan_joint',
            'arm1_shoulder_lift_joint', 
            'arm1_elbow_joint',
            'arm1_wrist_1_joint',
            'arm1_wrist_2_joint',
            'arm1_wrist_3_joint'
        ]
        
        self.arm2_joints = [
            'arm2_shoulder_pan_joint',
            'arm2_shoulder_lift_joint',
            'arm2_elbow_joint', 
            'arm2_wrist_1_joint',
            'arm2_wrist_2_joint',
            'arm2_wrist_3_joint'
        ]
        
        # Service clients for collision checking
        self.state_validity_client = self.create_client(
            GetStateValidity, 
            '/check_state_validity'
        )
        
        # Action clients for trajectory execution
        self.arm1_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm1_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.arm2_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm2_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_state = None
        self.joints_received = Event()
        
        # Wait for services and action servers
        self.get_logger().info('Waiting for services and action servers...')
        self.state_validity_client.wait_for_service(timeout_sec=10.0)
        self.arm1_action_client.wait_for_server(timeout_sec=10.0)
        self.arm2_action_client.wait_for_server(timeout_sec=10.0)
        
        self.get_logger().info('All services and action servers ready!')
        
    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg
        if not self.joints_received.is_set():
            self.joints_received.set()
            
    def wait_for_joint_states(self, timeout=10.0):
        """Wait for joint states to be available"""
        if not self.joints_received.wait(timeout):
            raise RuntimeError("Failed to receive joint states within timeout")
            
    def create_robot_state(self, arm1_positions, arm2_positions):
        """Create a RobotState message with both arms positioned"""
        robot_state = RobotState()
        
        # Create joint state
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "world"
        
        # Combine joint names and positions
        joint_state.name = self.arm1_joints + self.arm2_joints
        joint_state.position = list(arm1_positions) + list(arm2_positions)
        
        robot_state.joint_state = joint_state
        return robot_state
        
    def check_collision_state(self, arm1_positions, arm2_positions):
        """Check if a given robot state results in collision"""
        if not self.state_validity_client.service_is_ready():
            self.get_logger().error('State validity service not ready')
            return None
            
        request = GetStateValidity.Request()
        request.robot_state = self.create_robot_state(arm1_positions, arm2_positions)
        request.group_name = "multi_arm"  # Group name from your MoveIt config
        
        try:
            future = self.state_validity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result():
                response = future.result()
                # Response.valid is False if there's a collision
                collision_detected = not response.valid
                if collision_detected:
                    self.get_logger().info(f'Collision detected! Contacts: {len(response.contacts)}')
                    for i, contact in enumerate(response.contacts):
                        self.get_logger().info(f'Contact {i}: {contact.body_name_1} <-> {contact.body_name_2}')
                else:
                    self.get_logger().info('No collision detected')
                return collision_detected
            else:
                self.get_logger().error('Service call failed')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error checking collision: {e}')
            return None
            
    def move_arms_to_positions(self, arm1_positions, arm2_positions, duration=3.0):
        """Move both arms to specified positions"""
        
        # Create trajectory for arm1
        arm1_traj = JointTrajectory()
        arm1_traj.header.stamp = self.get_clock().now().to_msg()
        arm1_traj.joint_names = self.arm1_joints
        
        arm1_point = JointTrajectoryPoint()
        arm1_point.positions = list(arm1_positions)
        arm1_point.time_from_start.sec = int(duration)
        arm1_point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        arm1_traj.points = [arm1_point]
        
        # Create trajectory for arm2
        arm2_traj = JointTrajectory()
        arm2_traj.header.stamp = self.get_clock().now().to_msg()
        arm2_traj.joint_names = self.arm2_joints
        
        arm2_point = JointTrajectoryPoint()
        arm2_point.positions = list(arm2_positions)
        arm2_point.time_from_start.sec = int(duration)
        arm2_point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        arm2_traj.points = [arm2_point]
        
        # Create goals
        arm1_goal = FollowJointTrajectory.Goal()
        arm1_goal.trajectory = arm1_traj
        
        arm2_goal = FollowJointTrajectory.Goal()
        arm2_goal.trajectory = arm2_traj
        
        # Send goals
        self.get_logger().info('Sending trajectory goals to both arms...')
        
        arm1_future = self.arm1_action_client.send_goal_async(arm1_goal)
        arm2_future = self.arm2_action_client.send_goal_async(arm2_goal)
        
        # Wait for acceptance
        rclpy.spin_until_future_complete(self, arm1_future, timeout_sec=2.0)
        rclpy.spin_until_future_complete(self, arm2_future, timeout_sec=2.0)
        
        arm1_handle = arm1_future.result()
        arm2_handle = arm2_future.result()
        
        if not arm1_handle.accepted:
            self.get_logger().error('Arm1 trajectory rejected')
            return False
            
        if not arm2_handle.accepted:
            self.get_logger().error('Arm2 trajectory rejected')
            return False
            
        # Wait for completion
        self.get_logger().info('Waiting for trajectory execution...')
        
        arm1_result_future = arm1_handle.get_result_async()
        arm2_result_future = arm2_handle.get_result_async()
        
        rclpy.spin_until_future_complete(self, arm1_result_future, timeout_sec=duration + 2.0)
        rclpy.spin_until_future_complete(self, arm2_result_future, timeout_sec=duration + 2.0)
        
        success = True
        if arm1_result_future.result().result.error_code != 0:
            self.get_logger().error(f'Arm1 execution failed: {arm1_result_future.result().result.error_code}')
            success = False
            
        if arm2_result_future.result().result.error_code != 0:
            self.get_logger().error(f'Arm2 execution failed: {arm2_result_future.result().result.error_code}')
            success = False
            
        return success
        
    def test_wrist_collision_sequence(self):
        """Test sequence that should cause wrist3 collision"""
        self.get_logger().info('Starting wrist collision test sequence...')
        
        # Wait for joint states
        self.wait_for_joint_states()
        
        # Define test positions - arms should be close to each other with wrist3 joints overlapping
        # Position 1: Arms in safe positions
        safe_arm1 = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        safe_arm2 = [3.14, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        self.get_logger().info('Testing safe positions...')
        collision1 = self.check_collision_state(safe_arm1, safe_arm2)
        assert collision1 is not None, "Failed to check collision state"
        assert not collision1, "Unexpected collision in safe positions"
        
        # Move to safe positions first
        self.get_logger().info('Moving to safe positions...')
        success = self.move_arms_to_positions(safe_arm1, safe_arm2)
        assert success, "Failed to move to safe positions"
        
        time.sleep(1)
        
        # Position 2: Arms positioned so wrist3 joints are very close/overlapping
        # Both arms pointing towards center with similar wrist orientations
        collision_arm1 = [1.57, -1.2, 0.5, -0.8, 0.0, 1.57]  # Arm1 pointing right-center
        collision_arm2 = [-1.57, -1.2, -0.5, -0.8, 0.0, -1.57]  # Arm2 pointing left-center
        
        self.get_logger().info('Testing potential collision positions...')
        collision2 = self.check_collision_state(collision_arm1, collision_arm2)
        assert collision2 is not None, "Failed to check collision state"
        
        if collision2:
            self.get_logger().info('SUCCESS: Collision detected as expected!')
        else:
            self.get_logger().warning('No collision detected in collision positions - trying closer positions')
            
            # Try even closer positions
            collision_arm1_close = [1.4, -1.0, 0.8, -1.2, 0.2, 1.8]
            collision_arm2_close = [-1.4, -1.0, -0.8, -1.2, -0.2, -1.8]
            
            collision3 = self.check_collision_state(collision_arm1_close, collision_arm2_close)
            assert collision3 is not None, "Failed to check collision state"
            
            if collision3:
                self.get_logger().info('SUCCESS: Collision detected with closer positions!')
            else:
                self.get_logger().error('No collision detected even with close positions')
                
        # Return to safe positions
        self.get_logger().info('Returning to safe positions...')
        success = self.move_arms_to_positions(safe_arm1, safe_arm2)
        assert success, "Failed to return to safe positions"
        
        self.get_logger().info('Wrist collision test completed successfully!')
        
        return collision2 or (collision3 if 'collision3' in locals() else False)


def test_launch_and_collision_detection():
    """Main test function"""
    rclpy.init()
    
    tester = WristCollisionTester()
    
    try:
        # Run the collision test sequence
        collision_detected = tester.test_wrist_collision_sequence()
        
        # Verify we can detect collisions
        assert collision_detected, "Failed to detect collision between wrist3 joints"
        
        print("✓ Collision detection test passed!")
        
    except Exception as e:
        tester.get_logger().error(f'Test failed: {e}')
        raise
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    test_launch_and_collision_detection()