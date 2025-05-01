import os
from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():
    declared_arguments = []
    # TODO: add a hook to prepend the interbotix_xsarm_descriptions package to the env var
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(get_package_share_directory('interbotix_xsarm_descriptions'), "meshes"), ':' +
            str(Path(get_package_share_directory('interbotix_xsarm_descriptions')).parent.resolve())
        ]
    )
    declared_arguments.append(ign_resource_path)
    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [os.path.join(get_package_share_directory('multi_arm_lab_sim_gazebo'), "worlds", "lab.sdf"), " -r", " -v", "4"]
        }.items()
    )
    declared_arguments.append(gz_launch_description_with_gui)
    # launch multiple UR
    ur_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("multi_arm_lab_sim_bringup"), "/launch/multiple_ur.launch.py"]
        ),
        launch_arguments={}.items()
    )
    declared_arguments.append(ur_launch_description)
    # launch xsarm
    xsarm_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("multi_arm_lab_sim_bringup"), "/launch/xsarm.launch.py"]
        ),
        launch_arguments={}.items()
    )
    declared_arguments.append(xsarm_launch_description)
    """
    declared_arguments.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ur_launch_description,
            on_exit=[xsarm_launch_description],
        )
    ))
    """
    return LaunchDescription(declared_arguments)