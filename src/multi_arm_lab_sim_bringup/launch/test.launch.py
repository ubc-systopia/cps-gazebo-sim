import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declare_description_package = DeclareLaunchArgument(
        "description_package",
        default_value="ur_description",
        description="Description package for the robot",
    )

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ur_simulation_gz"), "launch"),
                "/ur_sim_control.launch.py",
            ]
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "prefix": "arm1",
            "description_package": LaunchConfiguration("description_package"),
            "world_file": os.path.join(
                get_package_share_directory("multi_arm_lab_sim_gazebo"),
                "worlds",
                "lab.sdf",
            ),
        }.items(),
    )
    ur_sim_with_namespace = GroupAction(
        actions=[
            PushRosNamespace("arm1"),
            ur_sim,
        ]
    )

    return LaunchDescription(
        declared_arguments
        + [
            declare_description_package,
            ur_sim,
            # ur_sim_with_namespace
        ]
    )
