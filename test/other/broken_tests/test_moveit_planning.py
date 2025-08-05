#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import time

class MoveItTester(Node):
    def __init__(self):
        super().__init__('moveit_tester')
        
        # Create service clients
        self.plan_service = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.ik_service = self.create_client(GetPositionIK, '/compute_ik')
        
        # Publisher for joint states (to see current robot state)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.current_joint_state = None
        
        self.get_logger().info('MoveIt Tester initialized')
        
        # Wait for services
        self.get_logger().info('Waiting for MoveIt services...')
        self.plan_service.wait_for_service(timeout_sec=10.0)
        self.ik_service.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Services are ready!')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def test_joint_planning(self):
        """Test planning to a specific joint configuration"""
        self.get_logger().info('Testing joint space planning...')
        
        if self.current_joint_state is None:
            self.get_logger().warn('No current joint state received yet')
            return False

        # Create motion plan request
        req = GetMotionPlan.Request()
        req.motion_plan_request.workspace_parameters.header.frame_id = "arm1_base_link"
        req.motion_plan_request.group_name = "ur3e_arm"
        req.motion_plan_request.num_planning_attempts = 5
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.planner_id = "RRTConnect"
        
        # Set start state (current state)
        req.motion_plan_request.start_state.joint_state = self.current_joint_state
        
        # Define target joint values (a safe configuration)
        target_joints = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # Safe position
        joint_names = [
            'arm1_shoulder_pan_joint',
            'arm1_shoulder_lift_joint', 
            'arm1_elbow_joint',
            'arm1_wrist_1_joint',
            'arm1_wrist_2_joint',
            'arm1_wrist_3_joint'
        ]
        
        # Create goal constraints
        goal_constraints = Constraints()
        for i, (joint_name, target_value) in enumerate(zip(joint_names, target_joints)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        
        req.motion_plan_request.goal_constraints = [goal_constraints]
        
        # Call the planning service
        try:
            self.get_logger().info('Calling motion planning service...')
            future = self.plan_service.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.motion_plan_response.error_code.val == 1:  # SUCCESS
                    self.get_logger().info('✅ Planning SUCCESSFUL!')
                    self.get_logger().info(f'Found trajectory with {len(response.motion_plan_response.trajectory.joint_trajectory.points)} points')
                    return True
                else:
                    self.get_logger().error(f'❌ Planning FAILED with error code: {response.motion_plan_response.error_code.val}')
                    return False
            else:
                self.get_logger().error('❌ Service call failed')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Exception during planning: {str(e)}')
            return False

    def test_cartesian_planning(self):
        """Test planning to a Cartesian pose"""
        self.get_logger().info('Testing Cartesian planning...')
        
        # Create IK request first to get valid joint target
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = "ur3e_arm"
        ik_req.ik_request.robot_state.joint_state = self.current_joint_state if self.current_joint_state else JointState()
        
        # Define target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "arm1_base_link"
        target_pose.pose.position = Point(x=0.3, y=0.2, z=0.4)
        target_pose.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)  # 90 deg rotation
        
        ik_req.ik_request.pose_stamped = target_pose
        ik_req.ik_request.ik_link_name = "arm1_tool0"
        ik_req.ik_request.attempts = 5
        ik_req.ik_request.timeout.sec = 5
        
        try:
            self.get_logger().info('Testing inverse kinematics...')
            ik_future = self.ik_service.call_async(ik_req)
            rclpy.spin_until_future_complete(self, ik_future, timeout_sec=10.0)
            
            if ik_future.result() is not None:
                ik_response = ik_future.result()
                if ik_response.error_code.val == 1:  # SUCCESS
                    self.get_logger().info('✅ IK SUCCESSFUL!')
                    return True
                else:
                    self.get_logger().error(f'❌ IK FAILED with error code: {ik_response.error_code.val}')
                    return False
            else:
                self.get_logger().error('❌ IK service call failed')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Exception during IK: {str(e)}')
            return False

    def test_predefined_state(self):
        """Test planning to the predefined 'upright' state"""
        self.get_logger().info('Testing planning to predefined state...')
        
        if self.current_joint_state is None:
            self.get_logger().warn('No current joint state received yet')
            return False

        # Create motion plan request for the "upright" state defined in SRDF
        req = GetMotionPlan.Request()
        req.motion_plan_request.workspace_parameters.header.frame_id = "arm1_base_link"
        req.motion_plan_request.group_name = "ur3e_arm"
        req.motion_plan_request.num_planning_attempts = 5
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.planner_id = "RRTConnect"
        
        # Set start state
        req.motion_plan_request.start_state.joint_state = self.current_joint_state
        
        # Target the "upright" state from SRDF
        upright_joints = [0, -1.673, 0, -1.5243, 0, 0]  # From SRDF
        joint_names = [
            'arm1_shoulder_pan_joint',
            'arm1_shoulder_lift_joint', 
            'arm1_elbow_joint',
            'arm1_wrist_1_joint',
            'arm1_wrist_2_joint',
            'arm1_wrist_3_joint'
        ]
        
        # Create goal constraints
        goal_constraints = Constraints()
        for joint_name, target_value in zip(joint_names, upright_joints):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        
        req.motion_plan_request.goal_constraints = [goal_constraints]
        
        try:
            self.get_logger().info('Planning to upright state...')
            future = self.plan_service.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.motion_plan_response.error_code.val == 1:  # SUCCESS
                    self.get_logger().info('✅ Upright state planning SUCCESSFUL!')
                    return True
                else:
                    self.get_logger().error(f'❌ Upright state planning FAILED with error code: {response.motion_plan_response.error_code.val}')
                    return False
            else:
                self.get_logger().error('❌ Service call failed')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Exception during upright planning: {str(e)}')
            return False

    def run_tests(self):
        """Run all tests"""
        self.get_logger().info('=== Starting MoveIt Functionality Tests ===')
        
        # Wait a bit for joint states
        self.get_logger().info('Waiting for joint states...')
        start_time = time.time()
        while self.current_joint_state is None and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_joint_state is None:
            self.get_logger().error('❌ No joint states received - check if joint_state_broadcaster is running')
            return
        
        self.get_logger().info(f'✅ Received joint states for joints: {self.current_joint_state.name}')
        
        # Run tests
        tests = [
            ('Joint Space Planning', self.test_joint_planning),
            ('Predefined State Planning', self.test_predefined_state),
            ('Inverse Kinematics', self.test_cartesian_planning),
        ]
        
        results = []
        for test_name, test_func in tests:
            self.get_logger().info(f'\n--- {test_name} ---')
            try:
                result = test_func()
                results.append((test_name, result))
                time.sleep(1)  # Brief pause between tests
            except Exception as e:
                self.get_logger().error(f'❌ {test_name} failed with exception: {str(e)}')
                results.append((test_name, False))
        
        # Summary
        self.get_logger().info('\n=== TEST SUMMARY ===')
        passed = 0
        for test_name, result in results:
            status = '✅ PASSED' if result else '❌ FAILED'
            self.get_logger().info(f'{test_name}: {status}')
            if result:
                passed += 1
        
        self.get_logger().info(f'\nOverall: {passed}/{len(results)} tests passed')
        
        if passed == len(results):
            self.get_logger().info('🎉 ALL TESTS PASSED! MoveIt is working correctly!')
        elif passed > 0:
            self.get_logger().info('⚠️  Some tests passed - partial functionality confirmed')
        else:
            self.get_logger().error('❌ All tests failed - there may be configuration issues')


def main():
    rclpy.init()
    
    tester = MoveItTester()
    
    try:
        tester.run_tests()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        tester.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
