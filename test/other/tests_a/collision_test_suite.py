#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    Constraints, 
    JointConstraint,
    RobotState,
    PlanningScene,
    CollisionObject
)
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
import numpy as np
import time

class CollisionTestSuite(Node):
    def __init__(self):
        super().__init__('collision_test_suite')
        
        # Action clients for both arms (using correct group names)
        self.arm1_client = ActionClient(self, MoveGroup, '/ur3e_arm1_move_group')
        self.arm2_client = ActionClient(self, MoveGroup, '/ur3e_arm2_move_group')
        
        # Service clients for collision checking
        self.get_planning_scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        self.get_state_validity_client = self.create_client(
            GetStateValidity, '/check_state_validity'
        )
        
        # Joint state subscriber to get current state
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.current_joint_state = None
        self.test_results = []
        
        self.get_logger().info("Collision Test Suite initialized")

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def wait_for_services(self):
        """Wait for all required services to be available"""
        self.get_logger().info("Waiting for MoveIt services...")
        
        if not self.get_planning_scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Planning scene service not available")
            return False
            
        if not self.get_state_validity_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("State validity service not available")
            return False
            
        self.get_logger().info("All services available")
        return True

    def check_state_validity(self, joint_positions, group_name="arm1"):
        """Check if a joint state is valid (collision-free)"""
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        
        # Set joint names based on group (using actual joint names from /joint_states)
        if group_name == "ur3e_arm1":
            joint_names = ['arm1shoulder_pan_joint', 'arm1shoulder_lift_joint', 
                          'arm1elbow_joint', 'arm1wrist_1_joint', 
                          'arm1wrist_2_joint', 'arm1wrist_3_joint']
        else:  # ur3e_arm2
            joint_names = ['arm2_shoulder_pan_joint', 'arm2_shoulder_lift_joint',
                          'arm2_elbow_joint', 'arm2_wrist_1_joint',
                          'arm2_wrist_2_joint', 'arm2_wrist_3_joint']
        
        request.robot_state.joint_state.name = joint_names
        request.robot_state.joint_state.position = joint_positions
        request.group_name = group_name
        
        future = self.get_state_validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().valid
        return False

    def generate_test_configurations(self):
        """Generate test joint configurations that might cause collisions"""
        test_configs = []
        
        # Test 1: Arms reaching towards each other
        arm1_collision_config = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # Arm1 reaching right
        arm2_collision_config = [3.14, -1.57, -1.57, -1.57, 1.57, 0.0]  # Arm2 reaching left
        
        test_configs.append({
            'name': 'Arms reaching towards each other',
            'arm1_config': arm1_collision_config,
            'arm2_config': arm2_collision_config,
            'expected_collision': True
        })
        
        # Test 2: Safe home positions
        arm1_home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        arm2_home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        test_configs.append({
            'name': 'Safe home positions',
            'arm1_config': arm1_home,
            'arm2_config': arm2_home,
            'expected_collision': False
        })
        
        # Test 3: Random configurations that might collide
        for i in range(5):
            # Generate random joint angles
            arm1_random = np.random.uniform(-3.14, 3.14, 6).tolist()
            arm2_random = np.random.uniform(-3.14, 3.14, 6).tolist()
            
            test_configs.append({
                'name': f'Random configuration {i+1}',
                'arm1_config': arm1_random,
                'arm2_config': arm2_random,
                'expected_collision': None  # Unknown
            })
        
        return test_configs

    def run_collision_test(self, test_config):
        """Run a single collision test"""
        self.get_logger().info(f"Running test: {test_config['name']}")
        
        # Check arm1 configuration validity
        arm1_valid = self.check_state_validity(test_config['arm1_config'], "ur3e_arm1")
        
        # Check arm2 configuration validity  
        arm2_valid = self.check_state_validity(test_config['arm2_config'], "ur3e_arm2")
        
        # Check combined state validity (both arms together)
        # This requires creating a combined robot state
        combined_valid = self.check_combined_state_validity(
            test_config['arm1_config'], 
            test_config['arm2_config']
        )
        
        result = {
            'test_name': test_config['name'],
            'arm1_valid': arm1_valid,
            'arm2_valid': arm2_valid,
            'combined_valid': combined_valid,
            'expected_collision': test_config['expected_collision'],
            'collision_detected': not combined_valid
        }
        
        # Log results
        status = "✓ PASS" if combined_valid else "✗ COLLISION"
        self.get_logger().info(f"  {status} - Combined state valid: {combined_valid}")
        
        return result

    def check_combined_state_validity(self, arm1_config, arm2_config):
        """Check validity of combined state with both arms"""
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        
        # Combine all joint names and positions (using correct joint names)
        all_joint_names = [
            'arm1shoulder_pan_joint', 'arm1shoulder_lift_joint', 
            'arm1elbow_joint', 'arm1wrist_1_joint', 
            'arm1wrist_2_joint', 'arm1wrist_3_joint',
            'arm2_shoulder_pan_joint', 'arm2_shoulder_lift_joint',
            'arm2_elbow_joint', 'arm2_wrist_1_joint',
            'arm2_wrist_2_joint', 'arm2_wrist_3_joint'
        ]
        
        all_positions = arm1_config + arm2_config
        
        request.robot_state.joint_state.name = all_joint_names
        request.robot_state.joint_state.position = all_positions
        request.group_name = ""  # Check entire robot
        
        future = self.get_state_validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().valid
        return False

    def run_test_suite(self):
        """Run the complete collision test suite"""
        if not self.wait_for_services():
            return
        
        self.get_logger().info("Starting collision test suite...")
        
        test_configs = self.generate_test_configurations()
        
        for config in test_configs:
            result = self.run_collision_test(config)
            self.test_results.append(result)
            time.sleep(1)  # Small delay between tests
        
        self.print_summary()

    def print_summary(self):
        """Print test results summary"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("COLLISION TEST SUITE RESULTS")
        self.get_logger().info("="*60)
        
        collision_count = 0
        safe_count = 0
        
        for result in self.test_results:
            status = "COLLISION" if result['collision_detected'] else "SAFE"
            expected = ""
            if result['expected_collision'] is not None:
                expected_str = "COLLISION" if result['expected_collision'] else "SAFE"
                match = "✓" if (result['collision_detected'] == result['expected_collision']) else "✗"
                expected = f" (Expected: {expected_str} {match})"
            
            self.get_logger().info(f"{result['test_name']}: {status}{expected}")
            
            if result['collision_detected']:
                collision_count += 1
            else:
                safe_count += 1
        
        self.get_logger().info("-"*60)
        self.get_logger().info(f"Total tests: {len(self.test_results)}")
        self.get_logger().info(f"Collisions detected: {collision_count}")
        self.get_logger().info(f"Safe configurations: {safe_count}")
        self.get_logger().info("="*60)

def main():
    rclpy.init()
    
    collision_tester = CollisionTestSuite()
    
    try:
        collision_tester.run_test_suite()
    except KeyboardInterrupt:
        pass
    finally:
        collision_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()