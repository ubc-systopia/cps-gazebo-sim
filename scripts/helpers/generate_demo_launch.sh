#!/bin/bash
# Usage: generate_demo_launch.sh <package_name> <pkg_dir>

set -euo pipefail

if [ $# -lt 2 ]; then
  echo "Usage: $0 <package_name> <pkg_dir>" >&2
  exit 1
fi

PACKAGE_NAME="$1"
PKG_DIR="$2"

LAUNCH_PATH="${PKG_DIR}/launch/demo.launch.py"
mkdir -p "${PKG_DIR}/launch"


cat > "$LAUNCH_PATH" << PY
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

def _read_robot_names(pkg_share: str):
    try:
        robots_json = os.path.join(pkg_share, 'config', 'robots.json')
        with open(robots_json, 'r') as f:
            data = json.load(f)
        arms = data.get('robot_arms', [])
        names = [a.get('key') for a in arms if isinstance(a, dict) and a.get('key')]
        return names
    except Exception:
        return []

def generate_launch_description():
    ##### 1. MoveIt
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    moveit_cfg = (
        MoveItConfigsBuilder("${PACKAGE_NAME}", package_name='${PACKAGE_NAME}')
        .to_moveit_configs()
    )

    moveit_cfg.robot_description["use_sim_time"] = use_sim_time
    moveit_cfg.planning_scene_monitor["use_sim_time"] = use_sim_time
    moveit_cfg.trajectory_execution["use_sim_time"] = use_sim_time
    rsp_launch = generate_rsp_launch(moveit_cfg)
    # Use the main package Xacro to provide robot_description (avoids nested <robot> from wrapper)
    robot_xacro_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([FindPackageShare('${PACKAGE_NAME}'), 'urdf', '${PACKAGE_NAME}.urdf.xacro'])
    ])
    moveit_cfg.robot_description['robot_description'] = ParameterValue(robot_xacro_cmd, value_type=str)

    rsp_launch        = generate_rsp_launch(moveit_cfg)
    move_group_launch = generate_move_group_launch(moveit_cfg)

    #### 2. Ignition Gazebo server + optional GUI
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
        }.items(),
    )

    #### 3. Spawn the robot after a short delay
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name",  "${PACKAGE_NAME}",
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
            get_package_share_directory("${PACKAGE_NAME}"),
            "config", "${PACKAGE_NAME}.rviz"
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
    pkg_share = get_package_share_directory('${PACKAGE_NAME}')
    robot_names = _read_robot_names(pkg_share)

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

    return LaunchDescription([
        use_sim_time_arg,
        gz_sim,
        rsp_launch,
        spawn_entity,
        delayed_controllers,
        move_group_launch,
        rviz_node
    ])
PY

chmod +x "$LAUNCH_PATH"
echo "Wrote ${LAUNCH_PATH}"
