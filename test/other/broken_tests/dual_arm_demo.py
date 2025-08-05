#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import time
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    RobotState
)
from sensor_msgs.msg import JointState
import geometry_msgs.msg
from rclpy.action import ActionClient


class DualArmController(Node):
    def __init__(self):
        super().__init__('dual_arm_controller')
        
        # Create action clients for both arms
        self.arm1_client = ActionClient(self, MoveGroup, '/move_group')
        self.arm2_client = ActionClient(self, MoveGroup, '/move_group')
        
        # Wait for the action server to be available
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.arm1_client.wait_for_server()
        self.get_logger().info('MoveGroup action server available!')
        
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
        
    def create_joint_goal(self, group_name, joint_names, joint_values):
        """Create a motion plan request for joint space goals"""
        goal_msg = MoveGroup.Goal()
        
        # Set the request
        goal_msg.request.group_name = group_name
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Create joint constraints
        joint_constraints = []
        for i, (joint_name, value) in enumerate(zip(joint_names, joint_values)):
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = float(value)
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            joint_constraints.append(constraint)
        
        # Add constraints to goal constraints
        goal_constraints = Constraints()
        goal_constraints.joint_constraints = joint_constraints
        goal_msg.request.goal_constraints = [goal_constraints]
        
        # Planning options
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        return goal_msg
    
    async def move_arm1_to_position(self, joint_values):
        """Move arm1 to specified joint positions"""
        self.get_logger().info(f'Moving arm1 to position: {joint_values}')
        
        goal_msg = self.create_joint_goal('ur3e_arm1', self.arm1_joints, joint_values)
        
        # Send the goal
        send_goal_future = self.arm1_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Arm1 goal rejected')
            return False
            
        self.get_logger().info('Arm1 goal accepted')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Arm1 movement completed successfully')
            return True
        else:
            self.get_logger().error(f'Arm1 movement failed with error code: {result.error_code.val}')
            return False
    
    async def move_arm2_to_position(self, joint_values):
        """Move arm2 to specified joint positions"""
        self.get_logger().info(f'Moving arm2 to position: {joint_values}')
        
        goal_msg = self.create_joint_goal('ur3e_arm2', self.arm2_joints, joint_values)
        
        # Send the goal
        send_goal_future = self.arm2_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Arm2 goal rejected')
            return False
            
        self.get_logger().info('Arm2 goal accepted')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Arm2 movement completed successfully')
            return True
        else:
            self.get_logger().error(f'Arm2 movement failed with error code: {result.error_code.val}')
            return False
    
    async def run_demo_sequence(self):
        """Run a demo sequence moving both arms"""
        self.get_logger().info('Starting dual arm demo sequence...')
        
        # Define some demo positions (in radians)
        # Home position
        home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        # Position 1: Arms spread apart
        arm1_pos1 = [0.5, -1.2, 0.5, -1.57, 0.0, 0.0]
        arm2_pos1 = [-0.5, -1.2, -0.5, -1.57, 0.0, 0.0]
        
        # Position 2: Arms closer together
        arm1_pos2 = [0.2, -1.0, 0.3, -1.2, 0.1, 0.0]
        arm2_pos2 = [-0.2, -1.0, -0.3, -1.2, -0.1, 0.0]
        
        try:
            # Move to home position
            self.get_logger().info('Moving both arms to home position...')
            success1 = await self.move_arm1_to_position(home_position)
            success2 = await self.move_arm2_to_position(home_position)
            
            if not (success1 and success2):
                self.get_logger().error('Failed to move to home position')
                return
            
            time.sleep(2)
            
            # Move to position 1
            self.get_logger().info('Moving to position 1...')
            success1 = await self.move_arm1_to_position(arm1_pos1)
            success2 = await self.move_arm2_to_position(arm2_pos1)
            
            if not (success1 and success2):
                self.get_logger().error('Failed to move to position 1')
                return
                
            time.sleep(2)
            
            # Move to position 2
            self.get_logger().info('Moving to position 2...')
            success1 = await self.move_arm1_to_position(arm1_pos2)
            success2 = await self.move_arm2_to_position(arm2_pos2)
            
            if not (success1 and success2):
                self.get_logger().error('Failed to move to position 2')
                return
                
            time.sleep(2)
            
            # Return to home
            self.get_logger().info('Returning to home position...')
            success1 = await self.move_arm1_to_position(home_position)
            success2 = await self.move_arm2_to_position(home_position)
            
            if success1 and success2:
                self.get_logger().info('Demo sequence completed successfully!')
            else:
                self.get_logger().error('Failed to return to home position')
                
        except Exception as e:
            self.get_logger().error(f'Demo sequence failed: {str(e)}')


async def main():
    rclpy.init()
    
    controller = DualArmController()
    
    try:
        await controller.run_demo_sequence()
    except KeyboardInterrupt:
        controller.get_logger().info('Demo interrupted by user')
    except Exception as e:
        controller.get_logger().error(f'Demo failed: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import asyncio
    asyncio.run(main())
