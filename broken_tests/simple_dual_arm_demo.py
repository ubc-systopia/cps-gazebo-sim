#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import time
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import numpy as np


class SimpleDualArmController(Node):
    def __init__(self):
        super().__init__('simple_dual_arm_controller')
        
        # Initialize MoveItPy
        self.get_logger().info('Initializing MoveItPy...')
        self.moveit = MoveItPy(node_name="moveit_py_node")
        
        # Get planning components for both arms
        try:
            self.arm1_group = self.moveit.get_planning_component("ur3e_arm1")
            self.get_logger().info('Successfully got ur3e_arm1 planning component')
        except Exception as e:
            self.get_logger().error(f'Failed to get ur3e_arm1 planning component: {e}')
            self.arm1_group = None
            
        try:
            self.arm2_group = self.moveit.get_planning_component("ur3e_arm2") 
            self.get_logger().info('Successfully got ur3e_arm2 planning component')
        except Exception as e:
            self.get_logger().error(f'Failed to get ur3e_arm2 planning component: {e}')
            self.arm2_group = None
        
        # Get robot model
        self.robot_model = self.moveit.get_robot_model()
        
        # Joint names for each arm
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
        
    def move_arm_to_joint_positions(self, arm_group, joint_names, joint_positions):
        """Move an arm to specified joint positions"""
        if arm_group is None:
            self.get_logger().error('Arm group is None!')
            return False
            
        try:
            # Set joint value target
            arm_group.set_start_state_to_current_state()
            
            # Create a dictionary of joint positions
            joint_dict = dict(zip(joint_names, joint_positions))
            success = arm_group.set_goal_state(configuration_name="", joint_state_dict=joint_dict)
            
            if not success:
                self.get_logger().error('Failed to set goal state')
                return False
            
            # Plan and execute
            plan_result = arm_group.plan()
            
            if plan_result:
                self.get_logger().info('Planning successful, executing...')
                execute_result = arm_group.execute(True)  # True for blocking execution
                return execute_result
            else:
                self.get_logger().error('Planning failed')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error in move_arm_to_joint_positions: {e}')
            return False
    
    def run_demo_sequence(self):
        """Run a demo sequence moving both arms"""
        self.get_logger().info('Starting simple dual arm demo sequence...')
        
        # Define some demo positions (in radians)
        # Home position
        home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        # Position 1: Arms spread apart
        arm1_pos1 = [0.5, -1.2, 0.5, -1.0, 0.0, 0.0]
        arm2_pos1 = [-0.5, -1.2, -0.5, -1.0, 0.0, 0.0]
        
        # Position 2: Arms closer together  
        arm1_pos2 = [0.2, -1.0, 0.3, -0.8, 0.1, 0.0]
        arm2_pos2 = [-0.2, -1.0, -0.3, -0.8, -0.1, 0.0]
        
        try:
            # Move to home position
            self.get_logger().info('Moving both arms to home position...')
            if self.arm1_group:
                success1 = self.move_arm_to_joint_positions(self.arm1_group, self.arm1_joints, home_position)
                self.get_logger().info(f'Arm1 home move: {"SUCCESS" if success1 else "FAILED"}')
            
            if self.arm2_group:
                success2 = self.move_arm_to_joint_positions(self.arm2_group, self.arm2_joints, home_position)
                self.get_logger().info(f'Arm2 home move: {"SUCCESS" if success2 else "FAILED"}')
            
            time.sleep(2)
            
            # Move to position 1
            self.get_logger().info('Moving to position 1...')
            if self.arm1_group:
                success1 = self.move_arm_to_joint_positions(self.arm1_group, self.arm1_joints, arm1_pos1)
                self.get_logger().info(f'Arm1 pos1 move: {"SUCCESS" if success1 else "FAILED"}')
                
            if self.arm2_group:
                success2 = self.move_arm_to_joint_positions(self.arm2_group, self.arm2_joints, arm2_pos1)
                self.get_logger().info(f'Arm2 pos1 move: {"SUCCESS" if success2 else "FAILED"}')
                
            time.sleep(2)
            
            # Move to position 2
            self.get_logger().info('Moving to position 2...')
            if self.arm1_group:
                success1 = self.move_arm_to_joint_positions(self.arm1_group, self.arm1_joints, arm1_pos2)
                self.get_logger().info(f'Arm1 pos2 move: {"SUCCESS" if success1 else "FAILED"}')
                
            if self.arm2_group:
                success2 = self.move_arm_to_joint_positions(self.arm2_group, self.arm2_joints, arm2_pos2) 
                self.get_logger().info(f'Arm2 pos2 move: {"SUCCESS" if success2 else "FAILED"}')
                
            time.sleep(2)
            
            # Return to home
            self.get_logger().info('Returning to home position...')
            if self.arm1_group:
                success1 = self.move_arm_to_joint_positions(self.arm1_group, self.arm1_joints, home_position)
                self.get_logger().info(f'Arm1 return home: {"SUCCESS" if success1 else "FAILED"}')
                
            if self.arm2_group:
                success2 = self.move_arm_to_joint_positions(self.arm2_group, self.arm2_joints, home_position)
                self.get_logger().info(f'Arm2 return home: {"SUCCESS" if success2 else "FAILED"}')
            
            self.get_logger().info('Demo sequence completed!')
                
        except Exception as e:
            self.get_logger().error(f'Demo sequence failed: {str(e)}')


def main():
    rclpy.init()
    
    controller = SimpleDualArmController()
    
    try:
        # Wait a bit for everything to initialize
        time.sleep(2)
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


if __name__ == '__main__':
    main()
