# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl
import json
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def read_config(file_path):
    # Read the json file with robot arms
    with open(file_path, "r") as openfile:
        arms = json.load(openfile)["robot_arms"]
    if arms is None:
        raise Exception("No robot arms configuration found") 
    return arms

def get_brands(robot_arms_config):
    return [arm["brand"] for arm in robot_arms_config]

def get_arm_instances_by_brand(robot_arms_config: list[dict], brand: str) -> list[dict]:
    for arm in robot_arms_config:
        if arm["brand"] == brand:
            return arm["instances"]

def spawn_ur_robot(launch_nodes, robot_arm: dict, previous_final_action=None):
    namespace = f'/{robot_arm["name"]}'
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("multi_ur_description"), "urdf", "ur.urdf.xacro"]
            ),
            " ",
            "safety_limits:=",
            robot_arm["safety_limits"],
            " ",
            "safety_pos_margin:=",
            robot_arm["safety_pos_margin"],
            " ",
            "safety_k_position:=",
            robot_arm["safety_k_position"],
            " ",
            "name:=",
            robot_arm["name"],
            " ",
            "ur_type:=",
            robot_arm["ur_type"],
            " ",
            "prefix:=",
            robot_arm["prefix"],
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            robot_arm["initial_joint_controllers"],
        ]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    """rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )"""

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", f'{namespace}/controller_manager'],
    )

    # Delay rviz start after `joint_state_broadcaster`
    """delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )"""

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", f'{namespace}/controller_manager'],
        condition=IfCondition(robot_arm["start_joint_controller"]),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", f'{namespace}/controller_manager', "--stopped"],
        condition=UnlessCondition(robot_arm["start_joint_controller"]),
    )

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            f'{namespace}/robot_description',
            "-name",
            robot_arm["name"],
            "-allow_renaming",
            "true",
            "-x", robot_arm["x"],
            "-y", robot_arm["y"],
            "-z", robot_arm["z"],
            "-Y", robot_arm["Y"],
        ],
    )
    
    if previous_final_action:
        spawn_entity = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=previous_final_action,
                on_exit=[gz_spawn_entity],
            )
        )
    else:
        spawn_entity = gz_spawn_entity
    
    state_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[initial_joint_controller_spawner_started, initial_joint_controller_spawner_stopped],
        )
    )

    launch_nodes.append(robot_state_publisher_node)
    launch_nodes.append(spawn_entity)
    launch_nodes.append(state_controller_event)
    launch_nodes.append(arm_controller_event)

    return launch_nodes, joint_state_broadcaster_spawner


def launch_setup(context, *args, **kwargs):
    pkg_project_bringup = get_package_share_directory('multi_arm_lab_sim_bringup')
    pkg_project_gazebo = get_package_share_directory('multi_arm_lab_sim_gazebo')
    # Initialize Arguments
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package") # TODO: pass as arg to spawn robot so we can extend to other description packages (but get rid of as launch arg)
    description_file = LaunchConfiguration("description_file") # TODO: pass as arg to spawn robot so we can extend to other description packages (but get rid of as launch arg)
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller") # TODO: remove, unless using anything other than joint_trajectory_controller
    launch_rviz = LaunchConfiguration("launch_rviz")
    world_file = LaunchConfiguration("world_file")

    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [os.path.join(pkg_project_gazebo, "worlds", "lab.sdf"), " -r", " -v", "4"]
        }.items()
    )

    initial_joint_controllers = PathJoinSubstitution(
        [pkg_project_bringup, "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    # Read the json file with robot arms
    file_path = os.path.join(pkg_project_bringup, "config", "arms.json")
    arms_config = read_config(file_path=file_path)
    universal_robots = get_arm_instances_by_brand(arms_config, "Universal Robots")

    # Create a list of nodes to launch
    launch_nodes = []
    #launch_nodes.append(gz_launch_description_with_gui)

    previous_final_action = None
    for arm in universal_robots:
        print(arm["key"])
        arm_config = {
            "ur_type": arm["model"], # "ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "prefix": prefix,
            "start_joint_controller": start_joint_controller,
            "initial_joint_controllers": initial_joint_controllers,
            "x": arm["base_coordinates"]["x"],
            "y": arm["base_coordinates"]["y"],
            "z": arm["base_coordinates"]["z"],
            "Y": arm["base_coordinates"]["Y"],
            "name": arm["key"], # assumption that arm names are unique
            #"moveit_config_package": moveit_config_package
        }
        launch_nodes, previous_final_action = spawn_ur_robot(launch_nodes, arm_config, previous_final_action)

    return launch_nodes


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="multi_ur_description", # customized ur.urdf.xacro in this package to add namespace handling in gazebo plugin
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file (absolute path or filename from the gazebosim worlds collection) containing a custom world.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])