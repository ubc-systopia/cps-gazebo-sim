#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import time

class SimpleVerification(Node):
    def __init__(self):
        super().__init__('simple_verification')
        
        # Service clients
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        
        # Action client
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/ur3e_arm_controller/follow_joint_trajectory')
        
        # Subscriber
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.current_joints = None
        print("🔍 Simple MoveIt Verification Started")

    def joint_callback(self, msg):
        self.current_joints = msg

    def verify_system(self):
        """Verify key components are working"""
        
        print("\n" + "="*50)
        print("🎯 MOVEIT SYSTEM VERIFICATION")
        print("="*50)
        
        # 1. Joint States
        print("\n1️⃣  Testing Joint States...")
        start_time = time.time()
        while self.current_joints is None and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_joints:
            expected_joints = [
                'arm1_shoulder_pan_joint', 'arm1_shoulder_lift_joint', 'arm1_elbow_joint',
                'arm1_wrist_1_joint', 'arm1_wrist_2_joint', 'arm1_wrist_3_joint'
            ]
            found = [j for j in expected_joints if j in self.current_joints.name]
            print(f"   ✅ Joint states: {len(found)}/{len(expected_joints)} joints found")
            for joint in found:
                idx = self.current_joints.name.index(joint)
                pos = self.current_joints.position[idx]
                print(f"      🔧 {joint}: {pos:.3f} rad")
            joint_test = len(found) == len(expected_joints)
        else:
            print("   ❌ No joint states received")
            joint_test = False
        
        # 2. Planning Scene
        print("\n2️⃣  Testing Planning Scene...")
        if self.scene_client.wait_for_service(timeout_sec=3.0):
            try:
                req = GetPlanningScene.Request()
                future = self.scene_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                
                if future.result():
                    scene = future.result().scene
                    print(f"   ✅ Planning scene retrieved")
                    print(f"      🤖 Robot: {scene.robot_model_name}")
                    print(f"      🔗 Joints in state: {len(scene.robot_state.joint_state.name)}")
                    scene_test = True
                else:
                    print("   ❌ Failed to get planning scene")
                    scene_test = False
            except Exception as e:
                print(f"   ❌ Planning scene error: {e}")
                scene_test = False
        else:
            print("   ❌ Planning scene service not available")
            scene_test = False
        
        # 3. Motion Planning
        print("\n3️⃣  Testing Motion Planning...")
        if self.plan_client.wait_for_service(timeout_sec=3.0) and self.current_joints:
            try:
                req = GetMotionPlan.Request()
                req.motion_plan_request.group_name = "ur3e_arm"
                req.motion_plan_request.num_planning_attempts = 3
                req.motion_plan_request.allowed_planning_time = 3.0
                req.motion_plan_request.planner_id = "RRTConnect"
                
                # Workspace
                req.motion_plan_request.workspace_parameters.header.frame_id = "arm1_base_link"
                req.motion_plan_request.workspace_parameters.min_corner.x = -1.0
                req.motion_plan_request.workspace_parameters.min_corner.y = -1.0
                req.motion_plan_request.workspace_parameters.min_corner.z = -1.0
                req.motion_plan_request.workspace_parameters.max_corner.x = 1.0
                req.motion_plan_request.workspace_parameters.max_corner.y = 1.0
                req.motion_plan_request.workspace_parameters.max_corner.z = 1.0
                
                req.motion_plan_request.start_state.joint_state = self.current_joints
                
                # Simple goal: move shoulder by 0.3 radians
                current_pos = []
                joint_names = ['arm1_shoulder_pan_joint', 'arm1_shoulder_lift_joint', 'arm1_elbow_joint',
                              'arm1_wrist_1_joint', 'arm1_wrist_2_joint', 'arm1_wrist_3_joint']
                
                for name in joint_names:
                    if name in self.current_joints.name:
                        idx = self.current_joints.name.index(name)
                        current_pos.append(self.current_joints.position[idx])
                    else:
                        current_pos.append(0.0)
                
                target_pos = current_pos.copy()
                target_pos[0] += 0.3  # Move shoulder pan
                
                goal_constraints = Constraints()
                for name, pos in zip(joint_names, target_pos):
                    constraint = JointConstraint()
                    constraint.joint_name = name
                    constraint.position = float(pos)  # Ensure float type
                    constraint.tolerance_above = 0.01
                    constraint.tolerance_below = 0.01
                    constraint.weight = 1.0
                    goal_constraints.joint_constraints.append(constraint)
                
                req.motion_plan_request.goal_constraints = [goal_constraints]
                
                print(f"      🎯 Planning: shoulder {current_pos[0]:.3f} → {target_pos[0]:.3f} rad")
                future = self.plan_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
                
                if future.result():
                    response = future.result()
                    if response.motion_plan_response.error_code.val == 1:  # SUCCESS
                        num_points = len(response.motion_plan_response.trajectory.joint_trajectory.points)
                        if num_points > 0:
                            duration = response.motion_plan_response.trajectory.joint_trajectory.points[-1].time_from_start.sec
                            print(f"   ✅ Planning successful!")
                            print(f"      📊 Trajectory: {num_points} waypoints, {duration}s duration")
                            planning_test = True
                        else:
                            print("   ❌ Empty trajectory")
                            planning_test = False
                    else:
                        print(f"   ❌ Planning failed (error {response.motion_plan_response.error_code.val})")
                        planning_test = False
                else:
                    print("   ❌ Planning service call failed")
                    planning_test = False
            except Exception as e:
                print(f"   ❌ Planning error: {e}")
                planning_test = False
        else:
            print("   ❌ Planning service not available or no joint states")
            planning_test = False
        
        # 4. Controllers
        print("\n4️⃣  Testing Controllers...")
        if self.trajectory_client.wait_for_server(timeout_sec=3.0):
            print("   ✅ Trajectory controller action server available")
            controller_test = True
        else:
            print("   ❌ Trajectory controller not available")
            controller_test = False
        
        # Summary
        print("\n" + "="*50)
        print("📋 VERIFICATION SUMMARY")
        print("="*50)
        
        tests = [
            ("Joint States", joint_test),
            ("Planning Scene", scene_test),
            ("Motion Planning", planning_test),
            ("Controllers", controller_test)
        ]
        
        passed = sum(1 for _, result in tests if result)
        
        for name, result in tests:
            status = "✅ WORKING" if result else "❌ FAILED"
            print(f"{name:<15}: {status}")
        
        print("="*50)
        print(f"📊 Overall: {passed}/{len(tests)} components working")
        
        if passed == len(tests):
            print("🎉 ALL SYSTEMS GO! Your MoveIt setup is fully functional!")
            print("💡 Ready for:")
            print("   • Motion planning and execution")
            print("   • Interactive control (with working RViz)")
            print("   • Custom trajectory generation")
            print("   • Cartesian path planning")
        elif passed >= 3:
            print("✅ CORE SYSTEMS WORKING! Minor issues detected.")
            print("💡 Your setup can handle basic motion planning.")
        else:
            print("⚠️  ISSUES DETECTED. Please check failed components.")
        
        return passed == len(tests)

def main():
    rclpy.init()
    
    verifier = SimpleVerification()
    
    try:
        success = verifier.verify_system()
        if success:
            print("\n🚀 Next steps:")
            print("   • Try running RViz for interactive control")
            print("   • Create custom planning scripts")
            print("   • Test trajectory execution")
    except KeyboardInterrupt:
        print("\nVerification interrupted")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        verifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
