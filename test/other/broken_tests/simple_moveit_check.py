#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time

class SimpleMoveitTester(Node):
    def __init__(self):
        super().__init__('simple_moveit_tester')
        self.get_logger().info('Simple MoveIt Tester started')

def check_services():
    """Check what ROS services are available"""
    try:
        result = subprocess.run(['ros2', 'service', 'list'], 
                              capture_output=True, text=True, timeout=10)
        services = result.stdout.strip().split('\n')
        
        print("🔍 Available ROS Services:")
        moveit_services = [s for s in services if 'moveit' in s.lower() or 'plan' in s.lower() or 'ik' in s.lower()]
        if moveit_services:
            for service in moveit_services:
                print(f"  ✅ {service}")
        else:
            print("  ❌ No MoveIt-related services found")
            
        return moveit_services
    except Exception as e:
        print(f"❌ Error checking services: {e}")
        return []

def check_topics():
    """Check what ROS topics are available"""
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=10)
        topics = result.stdout.strip().split('\n')
        
        print("\n🔍 Available ROS Topics:")
        moveit_topics = [t for t in topics if any(keyword in t.lower() 
                        for keyword in ['joint', 'trajectory', 'moveit', 'plan'])]
        if moveit_topics:
            for topic in moveit_topics:
                print(f"  ✅ {topic}")
        else:
            print("  ❌ No MoveIt-related topics found")
            
        return moveit_topics
    except Exception as e:
        print(f"❌ Error checking topics: {e}")
        return []

def check_joint_states():
    """Check if joint states are being published"""
    try:
        print("\n🔍 Checking joint states...")
        result = subprocess.run(['ros2', 'topic', 'echo', '/joint_states', '--once'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0 and result.stdout:
            print("  ✅ Joint states are being published")
            # Parse joint names
            lines = result.stdout.split('\n')
            for line in lines:
                if 'name:' in line:
                    print(f"  📝 {line.strip()}")
                    break
            return True
        else:
            print("  ❌ No joint states received")
            return False
    except Exception as e:
        print(f"❌ Error checking joint states: {e}")
        return False

def test_move_group_interface():
    """Test if we can communicate with move_group"""
    try:
        print("\n🔍 Testing move_group communication...")
        # Try to get planning scene
        result = subprocess.run(['ros2', 'service', 'call', '/get_planning_scene', 
                               'moveit_msgs/srv/GetPlanningScene', '{}'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("  ✅ move_group is responding to service calls")
            return True
        else:
            print(f"  ❌ move_group service call failed: {result.stderr}")
            return False
    except Exception as e:
        print(f"❌ Error testing move_group: {e}")
        return False

def main():
    print("=== MoveIt System Status Check ===\n")
    
    # Check if ROS is running
    try:
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            print("🔍 Running ROS Nodes:")
            for node in nodes:
                if node.strip():
                    print(f"  📦 {node}")
            print()
        else:
            print("❌ No ROS nodes detected")
            return
    except Exception as e:
        print(f"❌ Error checking ROS nodes: {e}")
        return

    # Run checks
    services = check_services()
    topics = check_topics()
    joint_states_ok = check_joint_states()
    move_group_ok = test_move_group_interface()
    
    # Summary
    print("\n=== SUMMARY ===")
    print(f"📊 Services found: {len(services)}")
    print(f"📊 Topics found: {len(topics)}")
    print(f"🔧 Joint states: {'✅ Working' if joint_states_ok else '❌ Not working'}")
    print(f"🤖 move_group: {'✅ Working' if move_group_ok else '❌ Not working'}")
    
    if joint_states_ok and move_group_ok and services:
        print("\n🎉 MoveIt appears to be working correctly!")
        print("📝 Try using RViz or the MoveIt Python API for planning")
    else:
        print("\n⚠️  Some components may not be working correctly")
        
        if not joint_states_ok:
            print("  💡 Check that joint_state_broadcaster is running")
        if not move_group_ok:
            print("  💡 Check that move_group node is running")
        if not services:
            print("  💡 Check that MoveIt services are available")

if __name__ == '__main__':
    main()
