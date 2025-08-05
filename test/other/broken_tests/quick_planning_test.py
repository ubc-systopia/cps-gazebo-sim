#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class QuickPlanningTest(Node):
    def __init__(self):
        super().__init__('quick_planning_test')
        
        # Create service client
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        self.current_joints = None
        self.get_logger().info('Quick Planning Test started')

    def joint_callback(self, msg):
        self.current_joints = msg

    def test_simple_planning(self):
        """Test a simple joint planning request"""
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Planning service not available')
            return False
            
        if self.current_joints is None:
            self.get_logger().error('No joint states received')
            return False

        self.get_logger().info('Creating planning request...')
        
        # Create request
        req = GetMotionPlan.Request()
        
        # Basic request setup
        req.motion_plan_request.group_name = "ur3e_arm"
        req.motion_plan_request.num_planning_attempts = 3
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.planner_id = "RRTConnect"
        
        # Set workspace
        req.motion_plan_request.workspace_parameters.header.frame_id = "arm1_base_link"
        req.motion_plan_request.workspace_parameters.min_corner.x = -2.0
        req.motion_plan_request.workspace_parameters.min_corner.y = -2.0  
        req.motion_plan_request.workspace_parameters.min_corner.z = -2.0
        req.motion_plan_request.workspace_parameters.max_corner.x = 2.0
        req.motion_plan_request.workspace_parameters.max_corner.y = 2.0
        req.motion_plan_request.workspace_parameters.max_corner.z = 2.0
        
        # Set start state
        req.motion_plan_request.start_state.joint_state = self.current_joints
        
        # Define simple target (just move one joint slightly)
        joint_names = [
            'arm1_shoulder_pan_joint',
            'arm1_shoulder_lift_joint', 
            'arm1_elbow_joint',
            'arm1_wrist_1_joint',
            'arm1_wrist_2_joint',
            'arm1_wrist_3_joint'
        ]
        
        # Get current positions and modify slightly
        current_positions = []
        for name in joint_names:
            if name in self.current_joints.name:
                idx = self.current_joints.name.index(name)
                current_positions.append(self.current_joints.position[idx])
            else:
                current_positions.append(0.0)
        
        # Move first joint by 0.5 radians
        target_positions = current_positions.copy()
        target_positions[0] += 0.5  # Move shoulder_pan_joint
        
        # Create goal constraints
        goal_constraints = Constraints()
        for name, target_pos in zip(joint_names, target_positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = target_pos
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            goal_constraints.joint_constraints.append(constraint)
        
        req.motion_plan_request.goal_constraints = [goal_constraints]
        
        self.get_logger().info(f'Planning from {current_positions[0]:.3f} to {target_positions[0]:.3f} for shoulder_pan_joint')
        
        # Call service
        try:
            future = self.plan_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                error_code = response.motion_plan_response.error_code.val
                
                if error_code == 1:  # SUCCESS
                    num_points = len(response.motion_plan_response.trajectory.joint_trajectory.points)
                    duration = response.motion_plan_response.trajectory.joint_trajectory.points[-1].time_from_start.sec if num_points > 0 else 0
                    self.get_logger().info(f'✅ PLANNING SUCCESSFUL! Trajectory has {num_points} points, duration: {duration}s')
                    return True
                else:
                    self.get_logger().error(f'❌ Planning failed with error code: {error_code}')
                    return False
            else:
                self.get_logger().error('❌ No response from planning service')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Exception: {str(e)}')
            return False

def main():
    rclpy.init()
    
    tester = QuickPlanningTest()
    
    # Wait for joint states
    print("Waiting for joint states...")
    start_time = time.time()
    while tester.current_joints is None and (time.time() - start_time) < 10.0:
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    if tester.current_joints is None:
        print("❌ Failed to get joint states")
        return
    
    print(f"✅ Got joint states: {tester.current_joints.name}")
    
    # Test planning
    success = tester.test_simple_planning()
    
    if success:
        print("🎉 MoveIt planning is working correctly!")
    else:
        print("❌ Planning test failed")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
