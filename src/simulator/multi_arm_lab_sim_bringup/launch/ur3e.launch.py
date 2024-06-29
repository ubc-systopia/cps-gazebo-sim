'''Launch ur3e ignition_simulator with ros joint trajectory controller and state publisher'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('multi_arm_lab_sim_bringup')
    pkg_project_gazebo = get_package_share_directory('multi_arm_lab_sim_gazebo')
    pkg_project_description = get_package_share_directory('multi_arm_lab_sim_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'ur3e', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'ur3e.sdf'
        ])}.items(),
    )
   
    # parameter for ur3e controller
    joint_names_list=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                    "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
    ign_joint_topics_list=[]
    for joint_name in joint_names_list:
        ign_joint_topics_list.append("/model/ur3e/joint/%s/0/cmd_pos"%joint_name)
    
    # ros<-ign, joint state publisher for ur3e
    robot_state_publisher=Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
            {"joint_names": joint_names_list},
            {"ign_topic": "/world/default/model/ur3e/joint_state"},
        ]
    )
    # ros->ign,  joint controller for ur3e
    joint_controller=Node(package='robot_state_publisher', 
               executable='joint_controller',
               name="ur3e_joint_controller",
               parameters=[{"joint_names": joint_names_list},
                           {"ign_joint_topics": ign_joint_topics_list},
                           {"rate":200},
                          ],
               output='screen')
    
    # # Bridge ROS topics and Gazebo messages for establishing communication
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': os.path.join(pkg_project_bringup, 'config', 'ur3e_bridge.yaml'),
    #         'qos_overrides./tf_static.publisher.durability': 'transient_local',
    #     }],
    #     output='screen'
    # )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/model/ur3e/joint_state@'
            'sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/ur3e/pose@'
            'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/model/ur3e/pose', '/tf'),
            ('/world/default/model/ur3e/joint_state', '/joint_states')
        ]
    )

    # Visualize in RViz
    rviz = Node(
      package='rviz2',
      executable='rviz2',
      arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'tf_bridge.rviz')],
      condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        robot_state_publisher,
        #joint_controller,
        rviz
    ])