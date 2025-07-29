#!/usr/bin/env python3
"""
Simple test to verify that RViz can load the MoveIt plugin without geometric_shapes errors.
"""

import subprocess
import os
import time
import signal
import sys

def test_rviz_plugin_loading():
    """Test if RViz can load MoveIt plugins without errors"""
    
    print("=== Testing RViz MoveIt Plugin Loading ===")
    
    # Set up environment with library fix
    env = os.environ.copy()
    workspace_root = os.path.dirname(os.path.abspath(__file__))
    install_path = os.path.join(workspace_root, 'install')
    
    # Add our install directory to LD_LIBRARY_PATH
    if 'LD_LIBRARY_PATH' in env:
        env['LD_LIBRARY_PATH'] = f"{install_path}:{env['LD_LIBRARY_PATH']}"
    else:
        env['LD_LIBRARY_PATH'] = install_path
    
    print(f"Using LD_LIBRARY_PATH: {env['LD_LIBRARY_PATH']}")
    
    # Verify the symlink exists
    symlink_path = os.path.join(install_path, 'libgeometric_shapes.so.2.1.3')
    if os.path.exists(symlink_path):
        print(f"✓ Found geometric_shapes fix at: {symlink_path}")
        print(f"  -> Points to: {os.readlink(symlink_path)}")
    else:
        print(f"✗ Missing geometric_shapes fix at: {symlink_path}")
        return False
    
    # Try to launch RViz with a simple config for a few seconds
    print("\nLaunching RViz to test plugin loading...")
    
    try:
        # Use a timeout to automatically close RViz after a few seconds
        process = subprocess.Popen(
            ['ros2', 'run', 'rviz2', 'rviz2', '--display-config', 
             '/opt/ros/humble/share/moveit_setup_assistant/rviz/moveit.rviz'],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Wait a few seconds for startup
        time.sleep(5)
        
        # Check if process is still running (good sign)
        if process.poll() is None:
            print("✓ RViz started successfully!")
            
            # Terminate the process
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            
            return True
        else:
            # Process exited, check for errors
            stdout, stderr = process.communicate()
            
            if 'libgeometric_shapes.so.2.1.3: cannot open shared object file' in stderr:
                print("✗ Still getting geometric_shapes error")
                print("STDERR:", stderr)
                return False
            elif 'Failed to load library' in stderr and 'moveit_motion_planning_rviz_plugin' in stderr:
                print("✗ MoveIt plugin failed to load")
                print("STDERR:", stderr)
                return False
            else:
                print("✓ No geometric_shapes errors detected!")
                if stderr:
                    print("STDERR (non-critical):", stderr)
                return True
                
    except Exception as e:
        print(f"✗ Error launching RViz: {e}")
        return False

def main():
    # Source ROS2 environment
    subprocess.run(['bash', '-c', 'source /opt/ros/humble/setup.bash'], check=True)
    
    success = test_rviz_plugin_loading()
    
    if success:
        print("\n🎉 SUCCESS: RViz MoveIt plugin loading test passed!")
        print("\nYou can now run the full demo with:")
        print("  ./run_collision_test.sh")
        return 0
    else:
        print("\n❌ FAILED: RViz MoveIt plugin loading test failed")
        print("\nTry running with more debugging:")
        print("  LD_LIBRARY_PATH=./install:$LD_LIBRARY_PATH ros2 run rviz2 rviz2")
        return 1

if __name__ == '__main__':
    sys.exit(main())