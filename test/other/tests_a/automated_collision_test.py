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
import subprocess
import signal
import os
import atexit
from builtin_interfaces.msg import Duration

class AutomatedCollisionTest(Node):
    def __init__(self):
        super().__init__('automated_collision_test')
        
        # Store launch process
        self.launch_process = None
        
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
        
        # Register cleanup function
        atexit.register(self.cleanup)
        
        self.get_logger().info("Automated Collision Test Suite initialized")

    def cleanup(self):
        """Clean up launch process and all child processes on exit"""
        if self.launch_process:
            self.get_logger().info("Shutting down all processes...")
            
            # Kill the entire process group (including Gazebo, RViz, etc.)
            try:
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                self.get_logger().info("Sent SIGTERM to process group")
                
                # Wait a bit for graceful shutdown
                time.sleep(3)
                
                # Force kill if still running
                try:
                    os.killpg(os.getpgid(self.launch_process.pid), signal.SIGKILL)
                    self.get_logger().info("Force killed process group")
                except ProcessLookupError:
                    # Process group already terminated
                    pass
                    
            except ProcessLookupError:
                # Process already terminated
                self.get_logger().info("Process group already terminated")
            except Exception as e:
                self.get_logger().warn(f"Error during cleanup: {e}")
                
            # Also try to terminate the main process
            try:
                self.launch_process.terminate()
                self.launch_process.wait(timeout=5)
            except:
                try:
                    self.launch_process.kill()
                except:
                    pass

    def launch_demo(self):
        """Launch the demo automatically"""
        self.get_logger().info("Launching demo...")
        
        # Change to workspace directory
        workspace_dir = "/home/roman/code/cps-gazebo-sim-2"
        
        # Source ROS and workspace, then launch
        launch_cmd = [
            "bash", "-c", 
            f"source /opt/ros/humble/setup.bash && "
            f"cd {workspace_dir} && "
            f"source install/setup.bash && "
            f"ros2 launch two_arm_moveit_config demo.launch.py"
        ]
        
        try:
            self.launch_process = subprocess.Popen(
                launch_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            self.get_logger().info("Demo launch initiated...")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to launch demo: {e}")
            return False

    def wait_for_system_ready(self, timeout_sec=60):
        """Wait for the system to be fully ready"""
        self.get_logger().info("Waiting for system to be ready...")
        
        start_time = time.time()
        
        # Wait for services to be available
        services_ready = False
        while not services_ready and (time.time() - start_time) < timeout_sec:
            try:
                if (self.get_state_validity_client.wait_for_service(timeout_sec=1.0) and
                    self.arm1_trajectory_client.wait_for_server(timeout_sec=1.0) and
                    self.arm2_trajectory_client.wait_for_server(timeout_sec=1.0)):
                    services_ready = True
                else:
                    self.get_logger().info("Waiting for services...")
                    time.sleep(2)
            except:
                time.sleep(2)
        
        if not services_ready:
            self.get_logger().error("Services not ready within timeout")
            return False
        
        # Wait for joint states
        joint_states_ready = False
        while not joint_states_ready and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joint_state is not None:
                joint_states_ready = True
            else:
                self.get_logger().info("Waiting for joint states...")
                time.sleep(1)
        
        if not joint_states_ready:
            self.get_logger().error("Joint states not ready within timeout")
            return False
        
        self.get_logger().info("System is ready!")
        return True

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

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
        
        all_joint_names = self.arm1_joint_names + self.arm2_joint_names
        all_positions = arm1_config + arm2_config
        
        request.robot_state.joint_state.name = all_joint_names
        request.robot_state.joint_state.position = all_positions
        request.group_name = ""
        
        future = self.get_state_validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().valid
        return False

    def create_trajectory(self, joint_names, start_positions, target_positions, duration_sec=3.0):
        """Create a joint trajectory from start to target positions"""
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        start_point.velocities = [0.0] * len(joint_names)
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        
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
        
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        success = result.result.error_code == 0
        
        return success

    def move_to_home_position(self):
        """Move both arms to home position"""
        self.get_logger().info("Moving arms to home position...")
        
        arm1_home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        arm2_home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        arm1_current = self.get_current_joint_positions("arm1")
        arm2_current = self.get_current_joint_positions("arm2")
        
        if arm1_current is None or arm2_current is None:
            self.get_logger().error("Could not get current joint positions")
            return False
        
        arm1_traj = self.create_trajectory(self.arm1_joint_names, arm1_current, arm1_home, 4.0)
        arm2_traj = self.create_trajectory(self.arm2_joint_names, arm2_current, arm2_home, 4.0)
        
        arm1_success = self.execute_trajectory(self.arm1_trajectory_client, arm1_traj)
        arm2_success = self.execute_trajectory(self.arm2_trajectory_client, arm2_traj)
        
        return arm1_success and arm2_success

    def run_collision_scenario(self, name, arm1_target, arm2_target, expected_collision=None):
        """Run a single collision test scenario"""
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"RUNNING SCENARIO: {name}")
        self.get_logger().info(f"{'='*50}")
        
        collision_predicted = not self.check_state_validity(arm1_target, arm2_target)
        status = "COLLISION PREDICTED" if collision_predicted else "SAFE PREDICTED"
        self.get_logger().info(f"Pre-check: {status}")
        
        arm1_current = self.get_current_joint_positions("arm1")
        arm2_current = self.get_current_joint_positions("arm2")
        
        if arm1_current is None or arm2_current is None:
            return None
        
        self.get_logger().info("Moving arms to target positions - watch Gazebo!")
        
        arm1_traj = self.create_trajectory(self.arm1_joint_names, arm1_current, arm1_target, 5.0)
        arm2_traj = self.create_trajectory(self.arm2_joint_names, arm2_current, arm2_target, 5.0)
        
        arm1_success = self.execute_trajectory(self.arm1_trajectory_client, arm1_traj)
        arm2_success = self.execute_trajectory(self.arm2_trajectory_client, arm2_traj)
        
        time.sleep(2.0)
        
        result = {
            'name': name,
            'predicted_collision': collision_predicted,
            'arm1_success': arm1_success,
            'arm2_success': arm2_success,
            'expected_collision': expected_collision
        }
        
        self.test_results.append(result)
        time.sleep(3.0)
        return result

    def run_full_test_suite(self):
        """Run the complete automated test suite"""
        self.get_logger().info("Starting Automated Collision Test Suite")
        
        # Launch demo
        if not self.launch_demo():
            return False
        
        # Wait for system to be ready
        if not self.wait_for_system_ready():
            self.cleanup()
            return False
        
        # Move to home position
        if not self.move_to_home_position():
            self.get_logger().error("Failed to move to home position")
            self.cleanup()
            return False
        
        time.sleep(2.0)
        
        # Run test scenarios
        scenarios = [
            {
                'name': 'Safe Home Positions',
                'arm1_target': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'arm2_target': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'expected_collision': False
            },
            {
                'name': 'Arms Reaching Toward Each Other',
                'arm1_target': [0.0, -1.0, 1.5, -1.5, -1.5, 0.0],
                'arm2_target': [3.14, -1.0, -1.5, -1.5, 1.5, 0.0],
                'expected_collision': True
            },
            {
                'name': 'Arms Extended Outward',
                'arm1_target': [-1.57, -1.57, 0.0, -1.57, 0.0, 0.0],
                'arm2_target': [1.57, -1.57, 0.0, -1.57, 0.0, 0.0],
                'expected_collision': False
            }
        ]
        
        for scenario in scenarios:
            self.run_collision_scenario(
                scenario['name'],
                scenario['arm1_target'],
                scenario['arm2_target'],
                scenario['expected_collision']
            )
        
        # Return to home
        self.move_to_home_position()
        
        # Print results
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("TEST SUITE COMPLETED")
        self.get_logger().info("="*60)
        
        for result in self.test_results:
            status = "SUCCESS" if (result['arm1_success'] and result['arm2_success']) else "FAILED"
            self.get_logger().info(f"{result['name']}: {status}")
        
        return True

def main():
    rclpy.init()
    
    collision_tester = AutomatedCollisionTest()
    
    try:
        success = collision_tester.run_full_test_suite()
        
        if success:
            collision_tester.get_logger().info("Test suite completed successfully!")
        else:
            collision_tester.get_logger().error("Test suite failed!")
        
        # Wait a moment to see final results
        collision_tester.get_logger().info("Shutting down in 5 seconds...")
        time.sleep(5)
        
    except KeyboardInterrupt:
        collision_tester.get_logger().info("Interrupted by user...")
    finally:
        collision_tester.get_logger().info("Cleaning up and shutting down...")
        collision_tester.cleanup()
        collision_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()