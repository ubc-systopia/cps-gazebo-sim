import os
from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_project_gazebo = get_package_share_directory('multi_arm_lab_sim_gazebo')
    pkg_project_description = get_package_share_directory("interbotix_xsarm_descriptions")
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_project_description, "meshes"), ':' +
            str(Path(pkg_project_description).parent.resolve())
        ]
    )
    
    
    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [os.path.join(pkg_project_gazebo, "worlds", "lab.sdf"), " -r", " -v", "4"]
        }.items()
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("interbotix_xsarm_descriptions"), "urdf", "vx300s.urdf.xacro"]
            ),
            " ",
            "hardware_type:=sim_ignition",
            " ",
            "robot_name:=xarm"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="/xarm",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "xarm",
            "-allow_renaming",
            "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
            "-Y", "0.0",
        ],
    )

    return LaunchDescription([
        ign_resource_path,
        gz_launch_description_with_gui,
        robot_state_publisher_node,
        gz_spawn_entity
    ])