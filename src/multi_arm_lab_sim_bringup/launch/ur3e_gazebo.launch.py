from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

def launch_setup():
    pkg_ur_simulation = get_package_share_directory('ur_simulation_gz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_project_gazebo = get_package_share_directory('multi_arm_lab_sim_gazebo')
    ur_type = LaunchConfiguration('ur_type')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    R = LaunchConfiguration('R')
    P = LaunchConfiguration('P')
    Y = LaunchConfiguration('Y')

def generate_launch_description():
    # Paths to necessary directories


    return LaunchDescription([
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        #    launch_arguments={
        #        "gz_args": ["-r", "-v", "4", PathJoinSubstitution([
        #            pkg_project_gazebo,
        #            'worlds',
        #            'empty.sdf'
        #    ])]}.items()
        #),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_ur_simulation, 'launch', 'ur_sim_control.launch.py'])),
            launch_arguments={
                "ur_type": "ur3e"
            }.items()
        )
        # TODO: add x,y,z,R,P,Y to gz_spawn_entity in ur_sim_control.launch.py
        # TODO: point pkg_ur_simulation to the local one (maybe add it to cps_gazebo_sim too)
        # TODO: declare arguments with the position, then test this launch file
        # NOTE: Goal is to launch multiple UR robots, of any type, wherever the user specifies
        # TEST that the robots can be controled individually
        # THEN we can incorporate other robot models
    ])
