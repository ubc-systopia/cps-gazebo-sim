import json
import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

def spawn_xsarm_robot(launch_nodes, robot_arm: dict, previous_final_action=None):
    namespace = f'/{robot_arm["name"]}'
    xsarm_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_descriptions'),
                'launch',
                'xsarm_description.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_arm["model"],
            'robot_name': robot_arm["name"], # this launch arg is used as namespace id
            'use_joint_pub': 'true',
            'use_sim_time': 'true',
        }.items(),
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            f'{namespace}/controller_manager',
            'joint_state_broadcaster',
        ],
        #parameters=[{
        #    'use_sim_time': 'true',
        #}],
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            f'{namespace}/controller_manager',
            'arm_controller',
        ],
        #parameters=[{
        #    'use_sim_time': 'true',
        #}]
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            f'{namespace}/controller_manager',
            'gripper_controller',
        ],
        #parameters=[{
        #    'use_sim_time': 'true',
        #}]
    )

    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            '-topic',
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
                on_exit=[spawn_robot_node],
            )
        )
    else:
        spawn_entity = spawn_robot_node
    
    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[spawn_joint_state_broadcaster_node]
        )
    )

    # Arm and gripper controller nodes can be started in any order
    load_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node, spawn_gripper_controller_node]
        )
    )

    launch_nodes.append(xsarm_description_launch_include)
    launch_nodes.append(spawn_entity)
    launch_nodes.append(load_joint_state_broadcaster_event)
    launch_nodes.append(load_controllers_event)

    return launch_nodes, spawn_joint_state_broadcaster_node


def launch_setup(context, *args, **kwargs):
    pkg_project_bringup = get_package_share_directory('multi_arm_lab_sim_bringup')
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
   
    # Read the json file with robot arms
    file_path = os.path.join(pkg_project_bringup, "config", "arms.json")
    arms_config = read_config(file_path=file_path)
    interbotix_xsarm_robots = get_arm_instances_by_brand(arms_config, "Interbotix")
    print(interbotix_xsarm_robots)
    # Create a list of nodes to launch
    launch_nodes = [ign_resource_path]
    launch_nodes.append(gz_launch_description_with_gui)

    previous_final_action = None
    for arm in interbotix_xsarm_robots:
        print(arm["key"])
        arm_config = {
            "model": arm["model"], # e.g. vx300s (the values defined by Interbotix)
            "x": arm["base_coordinates"]["x"],
            "y": arm["base_coordinates"]["y"],
            "z": arm["base_coordinates"]["z"],
            "Y": arm["base_coordinates"]["Y"],
            "name": arm["key"], # assumption that arm names are unique
            #"moveit_config_package": moveit_config_package
        }
        launch_nodes, previous_final_action = spawn_xsarm_robot(launch_nodes, arm_config, previous_final_action)

    return launch_nodes


def generate_launch_description():
    declared_arguments = []
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
