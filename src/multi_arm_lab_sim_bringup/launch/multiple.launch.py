import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution


def launch_setup():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim') # use spawn model
    return None

def generate_launch_description():
    robots = [
            {'name': 'arm1', 'x_pose': '-1.5', 'y_pose': '-1.50', 'Y':'0.0'},
            {'name': 'arm2', 'x_pose': '-1.5', 'y_pose': '1.5', 'Y':'0.0'},
            {'name': 'arm3', 'x_pose': '1.5', 'y_pose': '-1.5', 'Y':'-3.14'},
            {'name': 'arm4', 'x_pose': '1.5', 'y_pose': '1.5', 'Y':'-3.14'},
            # …
            # …
    ]
    
    declared_arguments = []
    declared_arguments.append("x")
    IncludeLaunchDescription(

    )

