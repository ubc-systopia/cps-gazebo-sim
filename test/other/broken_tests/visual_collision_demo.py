#!/usr/bin/env python3
"""
Visual Collision Demonstration for Dual-Arm Robot System

This script demonstrates collision detection with visual feedback in RViz.
It moves the arms through collision and non-collision scenarios while
checking for collisions in real-time.

Author: GitHub Copilot Assistant  
"""

import rclpy
from rclpy.node import Node
import asyncio
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState, DisplayRobotState
from sensor_msgs.msg import JointState
import math
import time


class VisualCollisionDemo(Node):
    def __init__(self):
        super().__init__('visual_collision_demo')
        
        # Create service client for collision detection
        self.state_validity_client = self.create_client(GetStateValidity, '/check_state_validity')
        
        # Create publisher for robot state visualization
        self.robot_state_publisher = self.create_publisher(
            DisplayRobotState, 
            '/display_robot_state', 
            10
        )
        
        # Wait for collision detection services
        self.get_logger().info('🔍 Checking MoveIt collision detection service...')
        if self.state_validity_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('✅ Collision detection service available!')
        else:
            self.get_logger().error('❌ Collision detection service not available!')
            self.get_logger().error('   Make sure MoveIt is running with: ros2 launch arm1g_transmission_moveit_config demo.launch.py')
            return
        
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
        
    def create_robot_state(self, arm1_positions, arm2_positions):
        """Create a robot state with both arm positions"""
        robot_state = RobotState()
        joint_state = JointState()
        
        joint_state.name = self.arm1_joints + self.arm2_joints
        joint_state.position = list(arm1_positions) + list(arm2_positions)
        
        robot_state.joint_state = joint_state
        robot_state.is_diff = False
        
        return robot_state
    
    def publish_robot_state(self, arm1_positions, arm2_positions, collision_status="unknown"):
        """Publish robot state for visualization in RViz"""
        try:
            display_state = DisplayRobotState()
            display_state.state = self.create_robot_state(arm1_positions, arm2_positions)
            
            # Add collision highlighting based on status
            if collision_status == "collision":
                # In a real implementation, you might set highlight links here
                pass
            
            self.robot_state_publisher.publish(display_state)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing robot state: {e}')
    
    async def check_collision_and_visualize(self, arm1_positions, arm2_positions, description=""):
        """Check collision and publish visualization"""
        try:
            # Create the request
            request = GetStateValidity.Request()
            request.robot_state = self.create_robot_state(arm1_positions, arm2_positions)
            request.group_name = ''  # Check all groups
            
            # Call the service
            future = self.state_validity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response is None:
                self.get_logger().error('Failed to get collision check response')
                return True, 0, 0
            
            # Analyze collision details
            inter_arm_collisions = 0
            intra_arm_collisions = 0
            collision_details = []
            
            if not response.valid and response.contacts:
                for i, contact in enumerate(response.contacts):
                    try:
                        if hasattr(contact, 'body_name_1') and hasattr(contact, 'body_name_2'):
                            body1 = contact.body_name_1
                            body2 = contact.body_name_2
                        elif hasattr(contact, 'contact_body_1') and hasattr(contact, 'contact_body_2'):
                            body1 = contact.contact_body_1
                            body2 = contact.contact_body_2
                        else:
                            # Extract from string representation
                            contact_str = str(contact)
                            if 'contact_body_1' in contact_str and 'contact_body_2' in contact_str:
                                parts = contact_str.split("'")
                                body1 = parts[1] if len(parts) > 1 else "unknown"
                                body2 = parts[3] if len(parts) > 3 else "unknown"
                            else:
                                body1 = f"contact_{i}_body1"
                                body2 = f"contact_{i}_body2"
                    except Exception as e:
                        body1 = f"unknown_body1_{i}"
                        body2 = f"unknown_body2_{i}"
                    
                    # Determine if this is inter-arm collision
                    is_inter_arm = ((body1.startswith('arm1') and body2.startswith('arm2')) or 
                                   (body1.startswith('arm2') and body2.startswith('arm1')))
                    
                    if is_inter_arm:
                        inter_arm_collisions += 1
                        collision_details.append(f"🚨 INTER-ARM: {body1} ↔ {body2}")
                    else:
                        intra_arm_collisions += 1
                        collision_details.append(f"⚠️  Intra-arm: {body1} ↔ {body2}")
            
            # Publish visualization
            collision_status = "collision" if not response.valid else "safe"
            self.publish_robot_state(arm1_positions, arm2_positions, collision_status)
            
            # Log results
            status_emoji = "🚨" if inter_arm_collisions > 0 else ("⚠️" if not response.valid else "✅")
            self.get_logger().info(f'{status_emoji} {description}')
            
            if not response.valid:
                if inter_arm_collisions > 0:
                    self.get_logger().error(f'   INTER-ARM COLLISION! ({inter_arm_collisions} contacts)')
                    for detail in collision_details:
                        if "INTER-ARM" in detail:
                            self.get_logger().error(f'   {detail}')
                elif intra_arm_collisions > 0:
                    self.get_logger().warn(f'   Intra-arm collision only ({intra_arm_collisions} contacts)')
            else:
                self.get_logger().info('   Safe configuration')
            
            return not response.valid, inter_arm_collisions, intra_arm_collisions
            
        except Exception as e:
            self.get_logger().error(f'Error in collision check: {e}')
            return True, 0, 0
    
    async def interpolate_motion(self, start_pos1, end_pos1, start_pos2, end_pos2, steps=20, description=""):
        """Interpolate between two arm configurations and check collisions"""
        self.get_logger().info(f'🎬 Starting motion: {description}')
        
        for i in range(steps + 1):
            t = i / steps  # Interpolation parameter (0 to 1)
            
            # Interpolate each joint
            current_pos1 = [start_pos1[j] + t * (end_pos1[j] - start_pos1[j]) for j in range(6)]
            current_pos2 = [start_pos2[j] + t * (end_pos2[j] - start_pos2[j]) for j in range(6)]
            
            step_description = f"Step {i+1}/{steps+1} ({t:.1f})"
            
            has_collision, inter_arm, intra_arm = await self.check_collision_and_visualize(
                current_pos1, current_pos2, step_description
            )
            
            # Brief pause for visualization
            await asyncio.sleep(0.5)
        
        self.get_logger().info(f'✅ Motion complete: {description}\n')
    
    async def run_visual_demo(self):
        """Run visual collision demonstration"""
        self.get_logger().info('🎬 Starting VISUAL collision demonstration...')
        self.get_logger().info('Watch RViz to see the robot configurations!')
        self.get_logger().info('The robot state will be published to /display_robot_state')
        
        # Wait a moment for RViz to be ready
        await asyncio.sleep(2.0)
        
        # Demo scenarios with smooth transitions
        demos = [
            {
                'name': 'Safe separation to close proximity',
                'start_arm1': [1.0, -1.2, 0.5, -1.57, 0.0, 0.0],
                'end_arm1': [0.2, -1.1, 0.4, -1.3, 0.0, 0.0],
                'start_arm2': [-1.0, -1.2, -0.5, -1.57, 0.0, 0.0],
                'end_arm2': [-0.2, -1.1, -0.4, -1.3, 0.0, 0.0],
                'steps': 15
            },
            {
                'name': 'Return to safe home positions',
                'start_arm1': [0.2, -1.1, 0.4, -1.3, 0.0, 0.0],
                'end_arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'start_arm2': [-0.2, -1.1, -0.4, -1.3, 0.0, 0.0],
                'end_arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'steps': 10
            },
            {
                'name': 'Both arms extending forward (collision expected)',
                'start_arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'end_arm1': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'start_arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'end_arm2': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'steps': 12
            },
            {
                'name': 'Return to safe configuration',
                'start_arm1': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'end_arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'start_arm2': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'end_arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'steps': 10
            }
        ]
        
        for i, demo in enumerate(demos, 1):
            self.get_logger().info(f'🎯 Demo {i}/{len(demos)}: {demo["name"]}')
            
            await self.interpolate_motion(
                demo['start_arm1'], demo['end_arm1'],
                demo['start_arm2'], demo['end_arm2'],
                demo['steps'], demo['name']
            )
            
            # Pause between demos
            await asyncio.sleep(1.0)
        
        self.get_logger().info('🏁 Visual collision demonstration complete!')
        self.get_logger().info('The robot should now be in a safe home configuration.')


async def main():
    rclpy.init()
    
    demo = VisualCollisionDemo()
    
    try:
        await demo.run_visual_demo()
    except KeyboardInterrupt:
        demo.get_logger().info('Visual demo interrupted by user')
    except Exception as e:
        demo.get_logger().error(f'Visual demo failed: {str(e)}')
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
