#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import time
from moveit_msgs.srv import GetPlanningScene, GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState, DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


class BasicDualArmController(Node):
    def __init__(self):
        super().__init__('basic_dual_arm_controller')
        
        # Publisher for joint states (to command the robot)
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        # Define joint names for both arms
        self.arm1_joints = [
            'arm1shoulder_pan_joint',
            'arm1shoulder_lift_joint',
            'arm1elbow_joint',
            'arm1wrist_1_joint',
            'arm1wrist_2_joint', 
            'arm1wrist_3_joint'
        ]
        
        self.arm2_joints = [
            'arm2_shoulder_pan_joint',
            'arm2_shoulder_lift_joint',
            'arm2_elbow_joint',
            'arm2_wrist_1_joint',
            'arm2_wrist_2_joint',
            'arm2_wrist_3_joint'
        ]
        
        self.get_logger().info('Basic dual arm controller initialized')
        
    def publish_joint_positions(self, arm1_positions, arm2_positions):
        """Publish joint positions for both arms"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Combine joint names and positions
        joint_state.name = self.arm1_joints + self.arm2_joints
        joint_state.position = list(arm1_positions) + list(arm2_positions)
        
        # Set velocities to zero
        joint_state.velocity = [0.0] * len(joint_state.name)
        
        self.joint_state_publisher.publish(joint_state)
        self.get_logger().info(f'Published joint positions: arm1={arm1_positions}, arm2={arm2_positions}')
        
    def interpolate_positions(self, start_pos, end_pos, steps=50):
        """Interpolate between two joint positions"""
        interpolated = []
        for i in range(steps + 1):
            t = i / steps
            pos = []
            for s, e in zip(start_pos, end_pos):
                pos.append(s + t * (e - s))
            interpolated.append(pos)
        return interpolated
        
    def move_arms_smoothly(self, arm1_start, arm1_end, arm2_start, arm2_end, duration=3.0):
        """Move both arms smoothly from start to end positions"""
        steps = 50
        arm1_trajectory = self.interpolate_positions(arm1_start, arm1_end, steps)
        arm2_trajectory = self.interpolate_positions(arm2_start, arm2_end, steps)
        
        time_per_step = duration / steps
        
        for i in range(len(arm1_trajectory)):
            self.publish_joint_positions(arm1_trajectory[i], arm2_trajectory[i])
            time.sleep(time_per_step)
    
    def run_demo_sequence(self):
        """Run a demo sequence moving both arms"""
        self.get_logger().info('Starting basic dual arm demo sequence...')
        
        # Define some demo positions (in radians)
        # Home position - both arms in neutral pose
        arm1_home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        arm2_home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        # Position 1: Arms spread apart horizontally
        arm1_pos1 = [0.8, -1.2, 0.5, -1.0, 0.0, 0.0]   # Arm1 to the right
        arm2_pos1 = [-0.8, -1.2, -0.5, -1.0, 0.0, 0.0]  # Arm2 to the left
        
        # Position 2: Arms closer together, raised up
        arm1_pos2 = [0.3, -0.8, 0.8, -0.5, 0.2, 0.0]
        arm2_pos2 = [-0.3, -0.8, -0.8, -0.5, -0.2, 0.0]
        
        # Position 3: Arms crossed over
        arm1_pos3 = [-0.2, -1.0, 0.3, -1.2, 0.0, 0.0]   # Arm1 crosses to left
        arm2_pos3 = [0.2, -1.0, -0.3, -1.2, 0.0, 0.0]   # Arm2 crosses to right
        
        try:
            # Start with home position
            self.get_logger().info('Moving to home position...')
            self.publish_joint_positions(arm1_home, arm2_home)
            time.sleep(2)
            
            # Move to position 1 - spread apart
            self.get_logger().info('Moving to spread position...')
            self.move_arms_smoothly(arm1_home, arm1_pos1, arm2_home, arm2_pos1, duration=3.0)
            time.sleep(1)
            
            # Move to position 2 - raised up
            self.get_logger().info('Moving to raised position...')
            self.move_arms_smoothly(arm1_pos1, arm1_pos2, arm2_pos1, arm2_pos2, duration=3.0)
            time.sleep(1)
            
            # Move to position 3 - crossed over
            self.get_logger().info('Moving to crossed position...')
            self.move_arms_smoothly(arm1_pos2, arm1_pos3, arm2_pos2, arm2_pos3, duration=3.0)
            time.sleep(1)
            
            # Return to home position
            self.get_logger().info('Returning to home position...')
            self.move_arms_smoothly(arm1_pos3, arm1_home, arm2_pos3, arm2_home, duration=3.0)
            time.sleep(1)
            
            # Wave motion
            self.get_logger().info('Performing wave motion...')
            for i in range(3):
                # Wave up
                wave_up = [j + math.sin(math.pi/4) * 0.3 for j in arm1_home]
                wave_up[4] = math.pi/2  # Rotate wrist
                self.move_arms_smoothly(arm1_home, wave_up, arm2_home, arm2_home, duration=1.0)
                
                # Wave down
                self.move_arms_smoothly(wave_up, arm1_home, arm2_home, arm2_home, duration=1.0)
            
            self.get_logger().info('Demo sequence completed successfully!')
                
        except Exception as e:
            self.get_logger().error(f'Demo sequence failed: {str(e)}')


def main():
    rclpy.init()
    
    controller = BasicDualArmController()
    
    try:
        # Wait a bit for everything to initialize
        controller.get_logger().info('Waiting for system to initialize...')
        time.sleep(3)
        
        controller.run_demo_sequence()
        
        # Keep the node alive for a bit after demo
        time.sleep(2)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Demo interrupted by user')
    except Exception as e:
        controller.get_logger().error(f'Demo failed: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
