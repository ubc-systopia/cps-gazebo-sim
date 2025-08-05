#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


def test_collision_service():
    rclpy.init()
    node = rclpy.create_node('simple_collision_test')
    
    # Create service client
    client = node.create_client(GetStateValidity, '/check_state_validity')
    
    print('Waiting for collision detection service...')
    if client.wait_for_service(timeout_sec=5.0):
        print('✅ Service available!')
        
        # Create a simple test request
        request = GetStateValidity.Request()
        robot_state = RobotState()
        joint_state = JointState()
        
        # Simple home position
        joint_state.name = [
            'arm1shoulder_pan_joint', 'arm1shoulder_lift_joint', 'arm1elbow_joint',
            'arm1wrist_1_joint', 'arm1wrist_2_joint', 'arm1wrist_3_joint',
            'arm2_shoulder_pan_joint', 'arm2_shoulder_lift_joint', 'arm2_elbow_joint',
            'arm2_wrist_1_joint', 'arm2_wrist_2_joint', 'arm2_wrist_3_joint'
        ]
        joint_state.position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0] * 2
        
        robot_state.joint_state = joint_state
        robot_state.is_diff = False
        request.robot_state = robot_state
        request.group_name = ''
        
        print('Calling collision service...')
        future = client.call_async(request)
        
        # Simple blocking wait
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            if response:
                print(f'✅ Service call successful!')
                print(f'   Valid: {response.valid}')
                print(f'   Contacts: {len(response.contacts) if response.contacts else 0}')
            else:
                print('❌ No response received')
        else:
            print('❌ Service call timed out')
    else:
        print('❌ Service not available')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    test_collision_service()
