"""
Robot collision detection and movement helper for dual-arm robot testing.
Provides MoveIt collision checking and trajectory execution capabilities.
"""

import time
from threading import Event
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class RobotCollisionHelper(Node):
    def __init__(self):
        super().__init__('collision_helper_node')
        
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
        
        self.state_validity_client = self.create_client(
            GetStateValidity, 
            '/check_state_validity'
        )
        
        self.arm1_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm1_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.arm2_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm2_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_state = None
        self.joints_received = Event()
        
    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        if not self.joints_received.is_set():
            self.joints_received.set()
            
    def wait_for_system_ready(self, timeout_sec=60):
        start_time = time.time()
        
        services_ready = False
        while not services_ready and (time.time() - start_time) < timeout_sec:
            try:
                if (self.state_validity_client.wait_for_service(timeout_sec=1.0) and
                    self.arm1_action_client.wait_for_server(timeout_sec=1.0) and
                    self.arm2_action_client.wait_for_server(timeout_sec=1.0)):
                    services_ready = True
                else:
                    time.sleep(2)
            except:
                time.sleep(2)
        
        if not services_ready:
            return False
        
        joint_states_ready = False
        while not joint_states_ready and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joint_state is not None:
                joint_states_ready = True
            else:
                time.sleep(1)
        
        if not joint_states_ready:
            return False
        
        return True
            
    def create_robot_state(self, arm1_positions, arm2_positions):
        robot_state = RobotState()
        
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "world"
        
        joint_state.name = self.arm1_joints + self.arm2_joints
        joint_state.position = list(arm1_positions) + list(arm2_positions)
        
        robot_state.joint_state = joint_state
        return robot_state
        
    def check_collision_state(self, arm1_positions, arm2_positions):
        if not self.state_validity_client.service_is_ready():
            return None
            
        request = GetStateValidity.Request()
        request.robot_state = self.create_robot_state(arm1_positions, arm2_positions)
        request.group_name = "multi_arm"
        
        try:
            future = self.state_validity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result():
                response = future.result()
                collision_detected = not response.valid
                
                colliding_pairs = set()
                collision_details = []
                
                if response.contacts:
                    for contact in response.contacts:
                        body1 = contact.contact_body_1
                        body2 = contact.contact_body_2
                        if body1 and body2:
                            pair = tuple(sorted([body1, body2]))
                            colliding_pairs.add(pair)
                            
                            collision_details.append({
                                'link1': body1,
                                'link2': body2,
                                'contact_position': [contact.position.x, contact.position.y, contact.position.z],
                                'contact_depth': contact.depth
                            })
                
                collision_count = len(colliding_pairs)
                
                return {
                    'collision_detected': collision_detected,
                    'collision_count': collision_count,
                    'colliding_pairs': list(colliding_pairs),
                    'collision_details': collision_details
                }
            else:
                return None
                
        except Exception as e:
            return None
            
    def get_current_joint_positions(self):
        if self.current_joint_state is None:
            return None, None
            
        arm1_positions = []
        arm2_positions = []
        
        for joint_name in self.arm1_joints:
            try:
                idx = self.current_joint_state.name.index(joint_name)
                arm1_positions.append(self.current_joint_state.position[idx])
            except ValueError:
                return None, None
        
        for joint_name in self.arm2_joints:
            try:
                idx = self.current_joint_state.name.index(joint_name)
                arm2_positions.append(self.current_joint_state.position[idx])
            except ValueError:
                return None, None
                
        return arm1_positions, arm2_positions

    def create_trajectory(self, joint_names, start_positions, target_positions, duration_sec=3.0):
        from builtin_interfaces.msg import Duration
        
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        start_point.velocities = [0.0] * len(joint_names)
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        
        end_point = JointTrajectoryPoint()
        end_point.positions = target_positions
        end_point.velocities = [0.0] * len(joint_names)
        end_point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        trajectory.points = [start_point, end_point]
        return trajectory

    def execute_trajectory(self, client, trajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        success = result.result.error_code == 0
        
        return success

    def move_arms_to_positions(self, arm1_positions, arm2_positions, duration=3.0):
        arm1_current, arm2_current = self.get_current_joint_positions()
        if arm1_current is None or arm2_current is None:
            return False
        
        arm1_traj = self.create_trajectory(self.arm1_joints, arm1_current, arm1_positions, duration)
        arm2_traj = self.create_trajectory(self.arm2_joints, arm2_current, arm2_positions, duration)
        
        arm1_success = self.execute_trajectory(self.arm1_action_client, arm1_traj)
        arm2_success = self.execute_trajectory(self.arm2_action_client, arm2_traj)
        
        return arm1_success and arm2_success