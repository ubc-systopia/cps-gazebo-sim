import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define namespaces for the robots
    namespace1 = 'ur3e_1'
    namespace2 = 'ur3e_2'

    # Path to the UR3e Gazebo launch file
    ur_gazebo_launch_file = os.path.join(
        get_package_share_directory('ur_simulation_gz'),
        'launch',
        'ur_sim_control.launch.py'
    )

    return LaunchDescription([
        GroupAction([
            PushRosNamespace(namespace1),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_gazebo_launch_file),
                launch_arguments={'namespace': namespace1}.items(),
            ),
        ]),
        GroupAction([
            PushRosNamespace(namespace2),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_gazebo_launch_file),
                launch_arguments={'namespace': namespace2}.items(),
            ),
        ]),
    ])
