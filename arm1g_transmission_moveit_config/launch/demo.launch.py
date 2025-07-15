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
    delayed_spawn = TimerAction(period=2.0, actions=[spawn_entity])

    # ── 4. Assemble and return ──────────────────────────────────────────────
    return LaunchDescription([
        gz_sim,
        rsp_launch,
        delayed_spawn,       # wait a bit, then insert robot
        move_group_launch,
    ])
