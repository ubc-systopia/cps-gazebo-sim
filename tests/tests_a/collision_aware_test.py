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

class CollisionAwareTest(Node):
    def __init__(self):
        super().__init__('collision_aware_test')
        
        self.launch_process = None
        
        # Action clients
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
        self.collision_detected = False
        
        # Joint names
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
        
        # Store active goal handles to cancel if collision detected
        self.active_arm1_goal = None
        self.active_arm2_goal = None
        
        atexit.register(self.cleanup)
        self.get_logger().info("Collision-Aware Test Suite initialized")

    def cleanup(self):
        """Clean up launch process and all child processes on exit"""
        if self.launch_process:
            self.get_logger().info("Shutting down all processes...")
            try:
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                time.sleep(3)
                try:
                    os.killpg(os.getpgid(self.launch_process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            except ProcessLookupError:
                pass
            except Exception as e:
                self.get_logger().warn(f"Error during cleanup: {e}")

    def launch_demo(self):
        """Launch the demo automatically"""
        self.get_logger().info("Launching demo...")
        workspace_dir = "/home/roman/code/cps-gazebo-sim-2"
        
        launch_cmd = [
            "bash", "-c", 
            f"source /opt/ros/humble/setup.bash && "
            f"cd {workspace_dir} && "
            f"source install/setup.bash && "
            f"ros2 launch arm1g_transmission_moveit_config demo.launch.py"
        ]
        
        try:
            self.launch_process = subprocess.Popen(
                launch_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to launch demo: {e}")
            return False

    def wait_for_system_ready(self, timeout_sec=60):
        """Wait for the system to be fully ready"""
        self.get_logger().info("Waiting for system to be ready...")
        start_time = time.time()
        
        # Wait for services
        services_ready = False
        while not services_ready and (time.time() - start_time) < timeout_sec:
            try:
                if (self.get_state_validity_client.wait_for_service(timeout_sec=1.0) and
                    self.arm1_trajectory_client.wait_for_server(timeout_sec=1.0) and
                    self.arm2_trajectory_client.wait_for_server(timeout_sec=1.0)):
                    services_ready = True
                else:
                    time.sleep(2)
            except:
                time.sleep(2)
        
        # Wait for joint states
        joint_states_ready = False
        while not joint_states_ready and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joint_state is not None:
                joint_states_ready = True
            else:
                time.sleep(1)
        
        return services_ready and joint_states_ready

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        
        # Check for collision during movement
        if self.active_arm1_goal or self.active_arm2_goal:
            if self.check_current_collision():
                if not self.collision_detected:
                    self.collision_detected = True
                    self.get_logger().warn("COLLISION DETECTED! Stopping trajectories...")
                    self.stop_all_trajectories()

    def check_current_collision(self):
        """Check if current robot state has collision"""
        if self.current_joint_state is None:
            return False
            
        try:
            # Get current positions for both arms
            arm1_positions = []
            arm2_positions = []
            
            for joint_name in self.arm1_joint_names:
                idx = self.current_joint_state.name.index(joint_name)
                arm1_positions.append(self.current_joint_state.position[idx])
                
            for joint_name in self.arm2_joint_names:
                idx = self.current_joint_state.name.index(joint_name)
                arm2_positions.append(self.current_joint_state.position[idx])
            
            return not self.check_state_validity(arm1_positions, arm2_positions)
        except:
            return False

    def stop_all_trajectories(self):
        """Cancel all active trajectory goals"""
        if self.active_arm1_goal:
            self.get_logger().info("Cancelling arm1 trajectory...")
            self.active_arm1_goal.cancel_goal_async()
            self.active_arm1_goal = None
            
        if self.active_arm2_goal:
            self.get_logger().info("Cancelling arm2 trajectory...")
            self.active_arm2_goal.cancel_goal_async()
            self.active_arm2_goal = None

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
                return None
                
        return positions

    def create_trajectory(self, joint_names, start_positions, target_positions, duration_sec=5.0):
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

    def execute_collision_aware_trajectory(self, arm1_target, arm2_target):
        """Execute trajectories with collision monitoring"""
        self.collision_detected = False
        
        # Get current positions
        arm1_current = self.get_current_joint_positions("arm1")
        arm2_current = self.get_current_joint_positions("arm2")
        
        if arm1_current is None or arm2_current is None:
            return False
        
        # Create trajectories
        arm1_traj = self.create_trajectory(self.arm1_joint_names, arm1_current, arm1_target, 8.0)
        arm2_traj = self.create_trajectory(self.arm2_joint_names, arm2_current, arm2_target, 8.0)
        
        # Send goals
        arm1_goal = FollowJointTrajectory.Goal()
        arm1_goal.trajectory = arm1_traj
        
        arm2_goal = FollowJointTrajectory.Goal()
        arm2_goal.trajectory = arm2_traj
        
        self.get_logger().info("Starting collision-aware movement...")
        
        # Send both goals
        arm1_future = self.arm1_trajectory_client.send_goal_async(arm1_goal)
        arm2_future = self.arm2_trajectory_client.send_goal_async(arm2_goal)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, arm1_future)
        rclpy.spin_until_future_complete(self, arm2_future)
        
        self.active_arm1_goal = arm1_future.result()
        self.active_arm2_goal = arm2_future.result()
        
        if not self.active_arm1_goal.accepted or not self.active_arm2_goal.accepted:
            self.get_logger().error("Goals not accepted")
            return False
        
        # Monitor execution
        arm1_result_future = self.active_arm1_goal.get_result_async()
        arm2_result_future = self.active_arm2_goal.get_result_async()
        
        # Spin until both complete or collision detected
        start_time = time.time()
        while (not arm1_result_future.done() or not arm2_result_future.done()) and not self.collision_detected:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Timeout after 10 seconds
            if time.time() - start_time > 10.0:
                self.get_logger().warn("Trajectory execution timeout")
                self.stop_all_trajectories()
                break
        
        # Clear active goals
        self.active_arm1_goal = None
        self.active_arm2_goal = None
        
        if self.collision_detected:
            self.get_logger().info("✓ COLLISION DETECTION WORKED! Trajectories stopped.")
            return "collision_detected"
        else:
            self.get_logger().info("Movement completed without collision")
            return "completed"

    def run_collision_test(self):
        """Run collision detection test"""
        if not self.launch_demo():
            return False
        
        if not self.wait_for_system_ready():
            self.cleanup()
            return False
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("COLLISION-AWARE MOVEMENT TEST")
        self.get_logger().info("="*60)
        
        # Test collision scenario
        self.get_logger().info("Testing arms reaching toward each other...")
        
        arm1_collision_target = [0.0, -1.0, 1.5, -1.5, -1.5, 0.0]  # Reach right
        arm2_collision_target = [3.14, -1.0, -1.5, -1.5, 1.5, 0.0]  # Reach left
        
        result = self.execute_collision_aware_trajectory(arm1_collision_target, arm2_collision_target)
        
        if result == "collision_detected":
            self.get_logger().info("✓ SUCCESS: Collision detection and trajectory stopping works!")
        elif result == "completed":
            self.get_logger().info("✗ No collision detected - arms may not have collided")
        else:
            self.get_logger().info("✗ Test failed")
        
        time.sleep(3)
        
        self.get_logger().info("Test completed. Shutting down in 5 seconds...")
        time.sleep(5)
        
        return True

def main():
    rclpy.init()
    
    collision_tester = CollisionAwareTest()
    
    try:
        collision_tester.run_collision_test()
    except KeyboardInterrupt:
        collision_tester.get_logger().info("Interrupted by user...")
    finally:
        collision_tester.cleanup()
        collision_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()