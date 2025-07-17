#!/usr/bin/env python3
#
# multi_arm_ignition.launch.py  – MoveIt 2 + Ignition Gazebo (blank world)
#
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch, generate_move_group_launch
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    # ── 1. MoveIt bits ───────────────────────────────────────────────────────
    moveit_cfg = (
        MoveItConfigsBuilder("mutli_arm",
                             package_name="arm1g_transmission_moveit_config")
        .to_moveit_configs()
    )
    rsp_launch        = generate_rsp_launch(moveit_cfg)
    move_group_launch = generate_move_group_launch(moveit_cfg)

    # ── 2. Ignition Gazebo server + optional GUI ─────────────────────────────
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            # String, not list → what gz_sim.launch.py expects
            "gz_args": "-r empty.sdf",   # start immediately (-r) with the built‑in blank world
            # "gui": "false",            # uncomment for headless
        }.items(),
    )

    # ── 3. Spawn the robot after a short delay ──────────────────────────────
    # A 2‑second timer is usually enough for /robot_description to appear.
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
    # common helper
    # def make_spawner(ns: str, controller: str):
    #     return Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         # namespace=ns,                                    # ➊ namespace
    #         arguments=[
    #             controller,
    #             "-c", f"multi_arm/{ns}/controller_manager"  # ➋ full path
    #         ],
    #         output="screen",
    #     )
    ### ---
    # def make_spawner(ns: str, controller: str):
    # # This now targets the new unique controller manager names, e.g., "arm1_controller_manager"
    # # We also add the controller name from the ros2_controllers.yaml file, e.g. "arm1_joint_trajectory_controller"
    # # We also add the controller name from the ros2_controllers.yaml file, e.g. "arm1_joint_trajectory_controller"
    #     if "joint_trajectory_controller" in controller:
    #         controller = ns + "_" + controller
    #     return Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=[
    #             controller,
    #             "-c", f"{ns}_controller_manager"
    #         ],
    #         output="screen",
    # )

    # spawners = [
    #     # arm 1
    #     make_spawner("arm1", "joint_state_broadcaster"),
    #     make_spawner("arm1", "joint_trajectory_controller"),
    #     # arm 2
    #     make_spawner("arm2", "joint_state_broadcaster"),
    #     make_spawner("arm2", "joint_trajectory_controller"),
    # ]
    # ---
    def make_spawner(manager: str, controller: str):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "-c", manager],
            output="screen",
        )
    
    unified_controller_manager = "multi_arm_controller_manager"

    # List of all controllers to be spawned
    spawners = [
        # Spawners for arm 1
        make_spawner(unified_controller_manager, "arm1_joint_state_broadcaster"),
        make_spawner(unified_controller_manager, "arm1_joint_trajectory_controller"),
        # Spawners for arm 2
        make_spawner(unified_controller_manager, "arm2_joint_state_broadcaster"),
        make_spawner(unified_controller_manager, "arm2_joint_trajectory_controller"),
    ]

    # Run all spawners two seconds after the entity is in the world
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=spawners,
        )
    )

    # ── 5. Assemble LD ───────────────────────────────────────────────────────
    return LaunchDescription([
        gz_sim,
        rsp_launch,
        spawn_entity,            # inserted immediately
        delayed_controllers,     # controllers 2 s later
        move_group_launch,
    ])