import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    # Paths to necessary directories
    ur_description_path = '/opt/ros/humble/share/ur_description'
    gazebo_ros_path = get_package_share_directory('ros_gz_sim')
    pkg_project_gazebo = get_package_share_directory('multi_arm_lab_sim_gazebo')
    # Path to the xacro file
    xacro_file = os.path.join(ur_description_path, 'urdf', 'ur.urdf.xacro')

    # Command to convert xacro to urdf
    robot_description = Command([
        'xacro ', xacro_file, 
        ' name:=ur3e',
        ' ur_type:=ur3e'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(gazebo_ros_path, 'launch', 'gz_sim.launch.py')]),
            launch_arguments={
                "gz_args": ["-r", "-v", "4", PathJoinSubstitution([
                    pkg_project_gazebo,
                    'worlds',
                    'thermoshaker.sdf'
                ])]
            }.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'ur3e', '-topic', '/robot_description'],
            output='screen'),
    ])
