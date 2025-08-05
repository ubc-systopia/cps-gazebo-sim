#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch, generate_move_group_launch
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    # ── 1. MoveIt bits ───────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    moveit_cfg = (
        MoveItConfigsBuilder("multi_arm",
                             package_name="arm1g_transmission_moveit_config")
        .to_moveit_configs()
    )

    moveit_cfg.robot_description["use_sim_time"] = use_sim_time
    moveit_cfg.planning_scene_monitor["use_sim_time"] = use_sim_time
    moveit_cfg.trajectory_execution["use_sim_time"] = use_sim_time

    rsp_launch        = generate_rsp_launch(moveit_cfg)
    move_group_launch = generate_move_group_launch(moveit_cfg)

    # ── 2. Ignition Gazebo server HEADLESS (no GUI) ─────────────────────────────
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-r -s empty.sdf",   # -s for server only (no GUI), -r to start immediately
            "gui": "false",                 # Explicitly disable GUI
        }.items(),
    )

    # ── 3. Spawn the robot after a short delay ──────────────────────────────
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name",  "multi_arm",
            "-allow_renaming", "true",
            "-x", "0", "-y", "0", "-z", "0.0",
        ],
    )

    # ── 4. ROS2‑control spawners (one per arm) ────────────────────────────────
    def make_spawner(manager: str, controller: str):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "-c", manager],
            output="screen",
        )

    # Use the default controller manager since that's what's actually working
    working_controller_manager = "controller_manager"

    # List of all controllers to be spawned - using the working controller manager
    spawners = [
        make_spawner(working_controller_manager, "arm1_joint_trajectory_controller"),
        make_spawner(working_controller_manager, "arm2_joint_trajectory_controller"),
        make_spawner(working_controller_manager, "joint_state_broadcaster"),
    ]

    # Run all spawners two seconds after the entity is in the world
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=spawners,
        )
    )

    # ── 5. Assemble Launch Description (NO RVIZ!) ───────────────────────────────────────────────────────
    return LaunchDescription([
        use_sim_time_arg,
        gz_sim,
        rsp_launch,
        spawn_entity,
        delayed_controllers,
        move_group_launch,
        # NOTE: RViz node removed for headless operation!
    ])