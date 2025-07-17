#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import moveit_commander
import sys
from geometry_msgs.msg import Pose
import time

class PlanningTest(Node):
    def __init__(self):
        super().__init__('planning_test')
        
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize the robot and scene
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Initialize the move group for the arm
        try:
            self.move_group = moveit_commander.MoveGroupCommander("ur3e_arm")
            self.get_logger().info("Successfully connected to ur3e_arm group")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to move group: {e}")
            return
            
        # Print current state
        self.get_logger().info(f"Planning frame: {self.move_group.get_planning_frame()}")
        self.get_logger().info(f"End effector link: {self.move_group.get_end_effector_link()}")
        self.get_logger().info(f"Group names: {self.robot.get_group_names()}")
        
        # Test planning to a simple goal
        self.test_planning()
        
    def test_planning(self):
        # Try planning to a named target first
        try:
            self.get_logger().info("Testing planning to named target 'upright'...")
            self.move_group.set_named_target("upright")
            plan = self.move_group.plan()
            
            if plan[0]:  # plan[0] is success boolean in ROS2
                self.get_logger().info("✅ Successfully planned to 'upright' position!")
            else:
                self.get_logger().warn("❌ Failed to plan to 'upright' position")
                
        except Exception as e:
            self.get_logger().error(f"Exception during planning: {e}")
            
        # Try planning to current position (should succeed)
        try:
            self.get_logger().info("Testing planning to current pose...")
            current_pose = self.move_group.get_current_pose().pose
            self.move_group.set_pose_target(current_pose)
            plan = self.move_group.plan()
            
            if plan[0]:
                self.get_logger().info("✅ Successfully planned to current pose!")
            else:
                self.get_logger().warn("❌ Failed to plan to current pose")
                
        except Exception as e:
            self.get_logger().error(f"Exception during pose planning: {e}")

def main():
    rclpy.init()
    
    # Wait a moment for MoveIt to be ready
    time.sleep(2)
    
    test_node = PlanningTest()
    
    try:
        rclpy.spin_once(test_node, timeout_sec=5.0)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
