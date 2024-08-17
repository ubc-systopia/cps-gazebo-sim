import json
import os
from pathlib import Path


from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from interbotix_xs_modules.xs_launch.xs_launch import determine_use_sim_time_param

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

def spawn_xsarm_robot(launch_nodes, robot_arm: dict, use_sim_time_param, previous_final_action=None):
    namespace = robot_arm["name"]
    
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

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            '-c',
            f'{namespace}/controller_manager',
            'joint_state_broadcaster',
        ],
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }],
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            '-c',
            'controller_manager',
            'arm_controller',
        ],
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }]
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            '-c',
            'controller_manager',
            'gripper_controller',
        ],
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }]
    )

    xsarm_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('xsarm_descriptions'),
                'launch',
                'xsarm_description.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_arm["model"],
            'robot_name': robot_arm["name"],
            'use_rviz': robot_arm["use_rviz_launch_arg"],
            'rvizconfig': robot_arm["rviz_config_launch_arg"],
            'use_sim_time': use_sim_time_param,
            'robot_description': robot_arm["robot_description_launch_arg"],
        }.items(),
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
    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]
        )
    )

    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_gripper_controller_node]
        )
    )

    launch_nodes.append(xsarm_description_launch_include)
    launch_nodes.append(spawn_entity)
    launch_nodes.append(load_joint_state_broadcaster_event)
    launch_nodes.append(load_arm_controller_event)
    launch_nodes.append(load_gripper_controller_event)

    return launch_nodes, spawn_joint_state_broadcaster_node


def launch_setup(context, *args, **kwargs):
    pkg_project_bringup = get_package_share_directory('multi_arm_lab_sim_bringup')
    pkg_project_gazebo = get_package_share_directory('multi_arm_lab_sim_gazebo')
    pkg_project_description = get_package_share_directory("xsarm_descriptions")

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_config_launch_arg = LaunchConfiguration('rvizconfig')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    
    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

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

    # Create a list of nodes to launch
    launch_nodes = [ign_resource_path]
    #launch_nodes.append(gz_launch_description_with_gui)

    previous_final_action = None
    for arm in interbotix_xsarm_robots:
        print(arm["key"])
        arm_config = {
            "model": arm["model"], # e.g. vx300s (the values defined by Interbotix)
            "use_rviz_launch_arg": use_rviz_launch_arg,
            "rviz_config_launch_arg": rviz_config_launch_arg,
            "robot_description_launch_arg": robot_description_launch_arg,
            "x": arm["base_coordinates"]["x"],
            "y": arm["base_coordinates"]["y"],
            "z": arm["base_coordinates"]["z"],
            "Y": arm["base_coordinates"]["Y"],
            "name": arm["key"], # assumption that arm names are unique
            #"moveit_config_package": moveit_config_package
        }
        launch_nodes, previous_final_action = spawn_xsarm_robot(launch_nodes, arm_config, use_sim_time_param, previous_final_action)

    return launch_nodes


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='vx300s',
            description='The robot model to launch.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_sim'),
                'rviz',
                'xsarm_gz_classic.rviz',
            ]),
            description='file path to the config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock.'
            )
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            hardware_type='sim_ignition',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
