"""
Gazebo launcher utility for managing headless Gazebo simulation during testing.
Provides functionality to launch, monitor, and cleanup Gazebo processes.
"""

import subprocess
import time
import psutil
import os
import signal
from typing import Optional


class GazeboLauncher:
    def __init__(self, launch_file_path: str):
        self.launch_file_path = launch_file_path
        self.gazebo_process: Optional[subprocess.Popen] = None
    
    def launch_gazebo(self) -> bool:
        try:
            workspace_dir = "/home/roman/code/cps-gazebo-sim-2"
            headless_launch_file = "/home/roman/code/cps-gazebo-sim-2/test/integration/collision_tests/utils/headless_demo.launch.py"
            
            launch_cmd = [
                "bash", "-c", 
                f"export DISPLAY= && "
                f"export LIBGL_ALWAYS_SOFTWARE=1 && "
                f"source /opt/ros/humble/setup.bash && "
                f"cd {workspace_dir} && "
                f"source install/setup.bash && "
                f"ros2 launch {headless_launch_file}"
            ]
            
            self.gazebo_process = subprocess.Popen(
                launch_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            time.sleep(5)
            
            if self.gazebo_process.poll() is not None:
                return False
                
            max_retries = 2
            for i in range(max_retries):
                if self.is_gazebo_running():
                    return True
                time.sleep(2)
            
            return True
            
        except Exception as e:
            return False
    
    def stop_gazebo(self):
        if self.gazebo_process:
            try:
                os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)
                try:
                    self.gazebo_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGKILL)
                    self.gazebo_process.wait()
            except ProcessLookupError:
                pass
            except Exception as e:
                pass
            finally:
                self.gazebo_process = None
        
        self._kill_remaining_processes()
        time.sleep(1)
    
    def _kill_remaining_processes(self):
        # Clean up any remaining simulation processes (including stray RViz from other sessions)
        process_names = ['gazebo', 'gzserver', 'gzclient', 'rviz2', 'rviz', 'ign']
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                proc_name = proc.info['name'].lower()
                cmdline = ' '.join(proc.info['cmdline']).lower() if proc.info['cmdline'] else ''
                
                if (any(name in proc_name for name in process_names) or 
                    'gazebo' in cmdline or 'rviz' in cmdline):
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except psutil.TimeoutExpired:
                        proc.kill()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
    
    def is_process_running(self, process_name: str) -> bool:
        for proc in psutil.process_iter(['name', 'cmdline']):
            try:
                proc_name = proc.info['name'].lower()
                cmdline = ' '.join(proc.info['cmdline']).lower() if proc.info['cmdline'] else ''
                
                if (process_name.lower() in proc_name or 
                    process_name.lower() in cmdline):
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return False
    
    def is_gazebo_running(self) -> bool:
        return (self.is_process_running('gazebo') or 
                self.is_process_running('gzserver') or 
                self.is_process_running('gzclient'))