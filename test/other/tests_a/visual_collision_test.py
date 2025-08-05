#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
import numpy as np
import time
from builtin_interfaces.msg import Duration

class VisualCollisionTest(Node):
    def __init__(self):
        super().__init__('visual_collision_test')
        
        # Action clients for trajectory execution
        self.arm1_trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm1_joint_trajectory_controller/follow_joint_trajectory'
        )
        self.arm2_trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm2_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Service client for collision checking
        self.get_state_validity_client = self.create_client(
            GetStateValidity, '/check_state_validity'
        )
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.current_joint_state = None
        self.test_results = []
        
        # Joint names for each arm
        self.arm1_joint_names = [
            'arm1shoulder_pan_joint', 'arm1shoulder_lift_joint', 
            'arm1elbow_joint', 'arm1wrist_1_joint', 
            'arm1wrist_2_joint', 'arm1wrist_3_joint'
        ]
        
        self.arm2_joint_names = [
            'arm2_shoulder_pan_joint', 'arm2_shoulder_lift_joint',
            'arm2_elbow_joint', 'arm2_wrist_1_joint',
            'arm2_wrist_2_joint', 'arm2_wrist_3_joint'
        ]
        
        self.get_logger().info("Visual Collision Test Suite initialized")

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def wait_for_services(self):
        """Wait for all required services to be available"""
        self.get_logger().info("Waiting for services and action servers...")
        
        if not self.get_state_validity_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("State validity service not available")
            return False
            
        if not self.arm1_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Arm1 trajectory action server not available")
            return False
            
        if not self.arm2_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Arm2 trajectory action server not available")
            return False
            
        self.get_logger().info("All services and action servers available")
        return True

    def get_current_joint_positions(self, arm_name):
        """Get current joint positions for specified arm"""
        if self.current_joint_state is None:
            return None
            
        joint_names = self.arm1_joint_names if arm_name == "arm1" else self.arm2_joint_names
        positions = []
        
        for joint_name in joint_names:
            try:
                idx = self.current_joint_state.name.index(joint_name)
                positions.append(self.current_joint_state.position[idx])
            except ValueError:
                self.get_logger().warn(f"Joint {joint_name} not found in joint states")
                return None
                
        return positions

    def check_state_validity(self, arm1_config, arm2_config):
        """Check if a combined joint state is valid (collision-free)"""
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        
        # Combine all joint names and positions
        all_joint_names = self.arm1_joint_names + self.arm2_joint_names
        all_positions = arm1_config + arm2_config
        
        request.robot_state.joint_state.name = all_joint_names
        request.robot_state.joint_state.position = all_positions
        request.group_name = ""  # Check entire robot
        
        future = self.get_state_validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().valid
        return False

    def create_trajectory(self, joint_names, start_positions, target_positions, duration_sec=3.0):
        """Create a joint trajectory from start to target positions"""
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        # Start point
        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        start_point.velocities = [0.0] * len(joint_names)
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        
        # End point
        end_point = JointTrajectoryPoint()
        end_point.positions = target_positions
        end_point.velocities = [0.0] * len(joint_names)
        end_point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        trajectory.points = [start_point, end_point]
        return trajectory

    def execute_trajectory(self, client, trajectory):
        """Execute a trajectory and wait for completion"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f"Sending trajectory with {len(trajectory.joint_names)} joints")
        
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            return False
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        success = result.result.error_code == 0
        
        if success:
            self.get_logger().info("Trajectory executed successfully")
        else:
            self.get_logger().warn(f"Trajectory execution failed with error code: {result.result.error_code}")
            
        return success

    def move_to_home_position(self):
        """Move both arms to home position"""
        self.get_logger().info("Moving arms to home position...")
        
        # Home positions (safe starting point)
        arm1_home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        arm2_home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        # Get current positions
        arm1_current = self.get_current_joint_positions("arm1")
        arm2_current = self.get_current_joint_positions("arm2")
        
        if arm1_current is None or arm2_current is None:
            self.get_logger().error("Could not get current joint positions")
            return False
        
        # Create and execute trajectories
        arm1_traj = self.create_trajectory(self.arm1_joint_names, arm1_current, arm1_home, 4.0)
        arm2_traj = self.create_trajectory(self.arm2_joint_names, arm2_current, arm2_home, 4.0)
        
        # Execute both trajectories simultaneously
        arm1_success = self.execute_trajectory(self.arm1_trajectory_client, arm1_traj)
        arm2_success = self.execute_trajectory(self.arm2_trajectory_client, arm2_traj)
        
        return arm1_success and arm2_success

    def run_collision_scenario(self, name, arm1_target, arm2_target, expected_collision=None):
        """Run a single collision test scenario with actual robot movement"""
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"RUNNING SCENARIO: {name}")
        self.get_logger().info(f"{'='*50}")
        
        # First, check if the target configuration would cause collision
        collision_predicted = not self.check_state_validity(arm1_target, arm2_target)
        
        status = "COLLISION PREDICTED" if collision_predicted else "SAFE PREDICTED"
        self.get_logger().info(f"Pre-check: {status}")
        
        if expected_collision is not None:
            expected_str = "COLLISION" if expected_collision else "SAFE"
            match = "✓" if (collision_predicted == expected_collision) else "✗"
            self.get_logger().info(f"Expected: {expected_str} {match}")
        
        # Get current positions
        arm1_current = self.get_current_joint_positions("arm1")
        arm2_current = self.get_current_joint_positions("arm2")
        
        if arm1_current is None or arm2_current is None:
            self.get_logger().error("Could not get current joint positions")
            return
        
        # Create trajectories to target positions
        self.get_logger().info("Moving arms to target positions...")
        
        arm1_traj = self.create_trajectory(self.arm1_joint_names, arm1_current, arm1_target, 5.0)
        arm2_traj = self.create_trajectory(self.arm2_joint_names, arm2_current, arm2_target, 5.0)
        
        # Execute trajectories (this is where we'll see any collisions in Gazebo)
        self.get_logger().info("Executing arm movements - watch Gazebo for collisions!")
        
        arm1_success = self.execute_trajectory(self.arm1_trajectory_client, arm1_traj)
        arm2_success = self.execute_trajectory(self.arm2_trajectory_client, arm2_traj)
        
        # Wait a moment to observe the result
        time.sleep(2.0)
        
        # Check final state validity
        final_arm1 = self.get_current_joint_positions("arm1")
        final_arm2 = self.get_current_joint_positions("arm2")
        
        if final_arm1 and final_arm2:
            final_valid = self.check_state_validity(final_arm1, final_arm2)
            final_status = "COLLISION DETECTED" if not final_valid else "NO COLLISION"
            self.get_logger().info(f"Final state: {final_status}")
        
        result = {
            'name': name,
            'predicted_collision': collision_predicted,
            'arm1_success': arm1_success,
            'arm2_success': arm2_success,
            'expected_collision': expected_collision
        }
        
        self.test_results.append(result)
        
        # Pause before next test
        self.get_logger().info("Pausing for 3 seconds before next test...")
        time.sleep(3.0)
        
        return result

    def run_test_suite(self):
        """Run the complete visual collision test suite"""
        if not self.wait_for_services():
            return
        
        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("Starting visual collision test suite...")
        
        # Move to home position first
        if not self.move_to_home_position():
            self.get_logger().error("Failed to move to home position")
            return
        
        time.sleep(2.0)
        
        # Test scenarios
        scenarios = [
            {
                'name': 'Safe Home Positions',
                'arm1_target': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'arm2_target': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'expected_collision': False
            },
            {
                'name': 'Arms Reaching Toward Each Other (High Collision Risk)',
                'arm1_target': [0.0, -1.0, 1.5, -1.5, -1.5, 0.0],  # Arm1 reaching right
                'arm2_target': [3.14, -1.0, -1.5, -1.5, 1.5, 0.0],  # Arm2 reaching left
                'expected_collision': True
            },
            {
                'name': 'Arms Extended Outward (Safe)',
                'arm1_target': [-1.57, -1.57, 0.0, -1.57, 0.0, 0.0],  # Arm1 left
                'arm2_target': [1.57, -1.57, 0.0, -1.57, 0.0, 0.0],   # Arm2 right
                'expected_collision': False
            },
            {
                'name': 'Arms in Center Space (Potential Collision)',
                'arm1_target': [0.5, -1.2, 1.0, -2.0, -1.0, 0.0],
                'arm2_target': [-0.5, -1.2, -1.0, -2.0, 1.0, 0.0],
                'expected_collision': True
            },
            {
                'name': 'Random Configuration Test',
                'arm1_target': np.random.uniform(-3.14, 3.14, 6).tolist(),
                'arm2_target': np.random.uniform(-3.14, 3.14, 6).tolist(),
                'expected_collision': None
            }
        ]
        
        for scenario in scenarios:
            self.run_collision_scenario(
                scenario['name'],
                scenario['arm1_target'],
                scenario['arm2_target'],
                scenario['expected_collision']
            )
        
        # Return to home position
        self.get_logger().info("Returning to home position...")
        self.move_to_home_position()
        
        self.print_summary()

    def print_summary(self):
        """Print test results summary"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("VISUAL COLLISION TEST SUITE RESULTS")
        self.get_logger().info("="*60)
        
        for result in self.test_results:
            status = "SUCCESS" if (result['arm1_success'] and result['arm2_success']) else "FAILED"
            
            expected = ""
            if result['expected_collision'] is not None:
                expected_str = "COLLISION" if result['expected_collision'] else "SAFE"
                predicted_str = "COLLISION" if result['predicted_collision'] else "SAFE"
                match = "✓" if (result['predicted_collision'] == result['expected_collision']) else "✗"
                expected = f" | Expected: {expected_str}, Predicted: {predicted_str} {match}"
            
            self.get_logger().info(f"{result['name']}: {status}{expected}")
        
        self.get_logger().info("="*60)
        self.get_logger().info("Test completed. Check Gazebo for visual collision results!")

def main():
    rclpy.init()
    
    collision_tester = VisualCollisionTest()
    
    try:
        collision_tester.run_test_suite()
    except KeyboardInterrupt:
        pass
    finally:
        collision_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()