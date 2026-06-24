#!/usr/bin/env python3
import os
import json

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch, generate_move_group_launch
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def _read_robots_json(pkg_share: str):
    try:
        with open(os.path.join(pkg_share, 'config', 'robots.json'), 'r') as f:
            return json.load(f)
    except Exception:
        return {}

def _read_robot_names(pkg_share: str):
    data = _read_robots_json(pkg_share)
    arms = data.get('robot_arms', [])
    return [a.get('key') for a in arms if isinstance(a, dict) and a.get('key')]

def _read_launch_rviz(pkg_share: str):
    # default True to preserve behaviour for older packages without the field
    return bool(_read_robots_json(pkg_share).get('launch_rviz', True))

def _read_static_objects(pkg_share: str):
    return _read_robots_json(pkg_share).get('static_objects') or []

def generate_launch_description():
    ##### 1. MoveIt
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    moveit_cfg = (
        MoveItConfigsBuilder("mesh_objects_demo", package_name='mesh_objects_demo')
        .to_moveit_configs()
    )

    moveit_cfg.robot_description["use_sim_time"] = use_sim_time
    moveit_cfg.planning_scene_monitor["use_sim_time"] = use_sim_time
    moveit_cfg.trajectory_execution["use_sim_time"] = use_sim_time
    rsp_launch = generate_rsp_launch(moveit_cfg)
    # Use the main package Xacro to provide robot_description (avoids nested <robot> from wrapper)
    robot_xacro_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('mesh_objects_demo'), 'urdf', 'mesh_objects_demo.urdf.xacro'])
    ])
    moveit_cfg.robot_description['robot_description'] = ParameterValue(robot_xacro_cmd, value_type=str)

    rsp_launch        = generate_rsp_launch(moveit_cfg)
    move_group_launch = generate_move_group_launch(moveit_cfg)

    #### 2. Ignition Gazebo server + optional GUI
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    # Use the generated world (with static objects) if it exists, else empty.sdf.
    _world = os.path.join(get_package_share_directory("mesh_objects_demo"), "config", "world.sdf")
    _gz_world = _world if os.path.exists(_world) else "empty.sdf"
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {_gz_world}",
        }.items(),
    )

    #### 3. Spawn the robot after a short delay
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name",  "mesh_objects_demo",
            "-allow_renaming", "true",
            "-x", "0", "-y", "0", "-z", "0.0",
        ],
    )

    rviz_node = Node (
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("mesh_objects_demo"),
            "config", "mesh_objects_demo.rviz"
        )],
        parameters=[
            moveit_cfg.robot_description,
            moveit_cfg.robot_description_semantic,
            {"robot_description": moveit_cfg.robot_description["robot_description"]},
            {"use_sim_time": use_sim_time},
        ],
    )

    #### 4. ROS2‑control spawners (one per arm)
    def make_spawner(manager: str, controller: str):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "-c", manager],
            output="screen",
        )

    # Use the default controller manager since that's what's actually working
    working_controller_manager = "controller_manager"

    # Build spawners for each robot found in robots.json, plus joint_state_broadcaster
    pkg_share = get_package_share_directory('mesh_objects_demo')
    robot_names = _read_robot_names(pkg_share)
    launch_rviz = _read_launch_rviz(pkg_share)

    spawners = [
        make_spawner(working_controller_manager, f"{name}_joint_trajectory_controller")
        for name in robot_names
    ]
    spawners.append(make_spawner(working_controller_manager, "joint_state_broadcaster"))

    # Run all spawners once the entity is in the world
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=spawners,
        )
    )

    #### 5. Populate the MoveIt planning scene with the static objects so the
    #### arms plan AROUND them. Delayed so move_group's /apply_planning_scene
    #### service is up; the node itself also waits on the service.
    has_static = bool(_read_static_objects(pkg_share))
    scene_publisher = Node(
        package="mesh_objects_demo",
        executable="scene_publisher.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    delayed_scene = TimerAction(period=8.0, actions=[scene_publisher])

    ld_entries = [
        use_sim_time_arg,
        gz_sim,
        rsp_launch,
        spawn_entity,
        delayed_controllers,
        move_group_launch,
    ]
    if launch_rviz:
        ld_entries.append(rviz_node)
    if has_static:
        ld_entries.append(delayed_scene)
    return LaunchDescription(ld_entries)
