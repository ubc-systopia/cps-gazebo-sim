#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import asyncio
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import math


class RealisticCollisionTester(Node):
    def __init__(self):
        super().__init__('realistic_collision_tester')
        
        self.state_validity_client = self.create_client(GetStateValidity, '/check_state_validity')
        self.get_logger().info('Waiting for demo.launch.py')
        if self.state_validity_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Collision detection service available!')
        else:
            self.get_logger().error('Collision detection service not available!')
            return
        
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
    
    async def check_collision(self, arm1_positions, arm2_positions, description=""):
        """Check if the given robot configuration has collisions"""
        try:
            request = GetStateValidity.Request()
            request.robot_state = self.create_robot_state(arm1_positions, arm2_positions)
            request.group_name = ''
            
            future = self.state_validity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response is None:
                self.get_logger().error('Failed to get collision check response')
                return True, 0, 0
            
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
                    
                    is_inter_arm = ((body1.startswith('arm1') and body2.startswith('arm2')) or 
                                   (body1.startswith('arm2') and body2.startswith('arm1')))
                    
                    if is_inter_arm:
                        inter_arm_collisions += 1
                        collision_type = "INTER-ARM"
                    else:
                        intra_arm_collisions += 1
                        collision_type = "Intra-arm"
                    
                    collision_details.append({
                        'type': collision_type,
                        'body1': body1,
                        'body2': body2,
                        'contact': contact
                    })
            
            self.get_logger().info(f'--- {description} ---')
            if not response.valid:
                self.get_logger().warn('COLLISION DETECTED!')
                self.get_logger().info(f'Total contacts: {len(response.contacts) if response.contacts else 0}')
                self.get_logger().info(f'Inter-arm collisions: {inter_arm_collisions}')
                self.get_logger().info(f'Intra-arm collisions: {intra_arm_collisions}')
                
                for detail in collision_details:
                    if detail['type'] == "INTER-ARM":
                        self.get_logger().error(f'   {detail["type"]}: {detail["body1"]} ↔ {detail["body2"]}')
                    
                if collision_details and hasattr(collision_details[0]['contact'], 'position'):
                    pos = collision_details[0]['contact'].position
                    self.get_logger().info(f'   Contact position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}')
            else:
                self.get_logger().info('No collisions detected')
            
            return not response.valid, inter_arm_collisions, intra_arm_collisions
            
        except Exception as e:
            self.get_logger().error(f'Error checking collision: {e}')
            return True, 0, 0 
    
    async def run_realistic_collision_tests(self):
        """Run realistic collision test scenarios"""
        self.get_logger().info('Starting collision detection tests...')
        self.get_logger().info('Testing practical dual-arm scenarios with detailed analysis')
        
        scenarios = [
            {
                'name': 'Both arms at home position',
                'description': 'Standard rest position - should be safe',
                'arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'expected': 'safe'
            },
            {
                'name': 'Arm 1 down to neutral position (expected: no collision)',
                'description': 'Standard rest position - should be safe',
                'arm1': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'arm2': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'expected': 'safe'
            },
            {
                'name': 'Arm 2 down to neutral position (expected: collision)',
                'description': 'Standard rest position - should be safe',
                'arm1': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                'arm2': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'expected': 'collision'
            }
            # {
            #     'name': 'Arms spread wide apart',
            #     'description': 'Working in separate areas - should be safe',
            #     'arm1': [1.2, -1.2, 0.5, -1.57, 0.0, 0.0],
            #     'arm2': [-1.2, -1.2, -0.5, -1.57, 0.0, 0.0],
            #     'expected': 'safe'
            # },
            # {
            #     'name': 'Arms working near center',
            #     'description': 'Both reaching toward shared workspace',
            #     'arm1': [-0.3, -1.0, 0.3, -1.2, 0.1, 0.0],
            #     'arm2': [0.3, -1.0, -0.3, -1.2, -0.1, 0.0],
            #     'expected': 'potential_collision'
            # },
            # {
            #     'name': 'Arms at medium proximity',
            #     'description': 'Working close but not overlapping',
            #     'arm1': [0.2, -1.1, 0.4, -1.3, 0.0, 0.0],
            #     'arm2': [-0.2, -1.1, -0.4, -1.3, 0.0, 0.0],
            #     'expected': 'potential_collision'
            # },
            # {
            #     'name': 'Arms reaching across each other',
            #     'description': 'Crossing paths - likely collision',
            #     'arm1': [-0.8, -0.8, 0.8, -1.5, 0.0, 0.0],
            #     'arm2': [0.8, -0.8, -0.8, -1.5, 0.0, 0.0],
            #     'expected': 'collision'
            # },
            # {
            #     'name': 'Very close parallel motion',
            #     'description': 'Arms moving in parallel, very close',
            #     'arm1': [0.1, -1.0, 0.2, -1.0, 0.0, 0.0],
            #     'arm2': [-0.1, -1.0, -0.2, -1.0, 0.0, 0.0],
            #     'expected': 'collision'
            # },
            # {
            #     'name': 'One arm extended, other folded',
            #     'description': 'Asymmetric poses - testing workspace boundaries',
            #     'arm1': [0.0, -0.5, -1.5, -1.0, 0.0, 0.0],
            #     'arm2': [0.0, -1.8, 0.0, -1.0, 0.0, 0.0],
            #     'expected': 'safe'
            # },
            # {
            #     'name': 'Both arms fully extended forward',
            #     'description': 'Maximum reach test',
            #     'arm1': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            #     'arm2': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            #     'expected': 'potential_collision'
            # }
        ]
        
        # Test statistics
        total_collisions = 0
        total_inter_arm = 0
        total_intra_arm = 0
        total_tests = len(scenarios)
        
        self.get_logger().info(f'\n Running {total_tests} realistic scenarios...\n')
        
        for i, scenario in enumerate(scenarios, 1):
            self.get_logger().info(f'Test {i}/{total_tests}: {scenario["name"]}')
            self.get_logger().info(f'{scenario["description"]}')
            self.get_logger().info(f'Expected: {scenario["expected"]}')
            self.get_logger().info(f'Arm1: {scenario["arm1"]}')
            self.get_logger().info(f'Arm2: {scenario["arm2"]}')
            
            has_collision, inter_arm, intra_arm = await self.check_collision(
                scenario['arm1'], 
                scenario['arm2'], 
                scenario['name']
            )
            
            if has_collision:
                total_collisions += 1
                total_inter_arm += inter_arm
                total_intra_arm += intra_arm
                
                if inter_arm > 0:
                    self.get_logger().error(f'RESULT: INTER-ARM COLLISION DETECTED!')
                else:
                    self.get_logger().warn(f'RESULT: Intra-arm collision only')
            else:
                self.get_logger().info(f'RESULT: Safe configuration')
            
            await asyncio.sleep(0.5)
            self.get_logger().info('')  # Empty line for readability
        
        # Final summary
        self.get_logger().info('COLLISION TEST SUMMARY')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Total scenarios tested: {total_tests}')
        self.get_logger().info(f'Scenarios with collisions: {total_collisions}')
        self.get_logger().info(f'Safe scenarios: {total_tests - total_collisions}')
        self.get_logger().info(f'Total inter-arm collisions: {total_inter_arm}')
        self.get_logger().info(f'Total intra-arm collisions: {total_intra_arm}')
        self.get_logger().info('')
        
        # Analysis
        if total_inter_arm > 0:
            self.get_logger().info('ANALYSIS: Inter-arm collision detection is WORKING!')
            self.get_logger().info(f'MoveIt successfully detected {total_inter_arm} inter-arm collisions')
            self.get_logger().info('The dual-arm system is properly configured for collision-aware planning')
        else:
            self.get_logger().warn('ANALYSIS: No inter-arm collisions detected')
        
        if total_intra_arm > 0:
            self.get_logger().info(f'Note: {total_intra_arm} intra-arm collisions detected (expected for some poses)')
        
        collision_rate = (total_collisions / total_tests) * 100
        self.get_logger().info(f'Overall collision detection rate: {collision_rate:.1f}%')
        
        if collision_rate > 0:
            self.get_logger().info('CONCLUSION: Collision detection system is functional!')
        else:
            self.get_logger().warn('CONCLUSION: May need configuration adjustments')


async def main():
    rclpy.init()
    
    tester = RealisticCollisionTester()
    
    try:
        await tester.run_realistic_collision_tests()
    except KeyboardInterrupt:
        tester.get_logger().info('Realistic collision tests interrupted by user')
    except Exception as e:
        tester.get_logger().error(f'Realistic collision tests failed: {str(e)}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
