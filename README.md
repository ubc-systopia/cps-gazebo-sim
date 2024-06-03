# cps-gazebo-sim

## System Requirements
- Ubuntu Jammy 22.04
- CMake version > 3.5

## Installation
[ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
> For convenience, add the following line to *.bashrc*. Other possible values are setup.sh, setup.zsh
>```bash
>source /opt/ros/humble/setup.bash
>```

[Gazebo Fortress](https://gazebosim.org/docs/fortress/install_ubuntu)
or by default as the recommended pairing for ROS2 Humble via the command:

```bash
sudo apt-get install ros-humble-ros-gz
```

### Drivers for Robots
|Robot|Installation Link|
|---|---|
|ViperX 300s|https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html|
|UR3e series|https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble|

## To run
https://gazebosim.org/docs/all/getstarted

```bash
ign gazebo shapes.sdf
```

Build
```bash
cd ~/cps-gazebo-sim
colcon build --cmake-args -DBUILD_TESTING=ON
```
Run
```bash
# source the workspace
. ~/cps-gazebo-sim/install/setup.sh
# launch the simulation and visualize in RViz
ros2 launch ros_gz_example_bringup diff_drive.launch.py
```

## All the technical steps that created this project
1. [Cloning a template](https://gazebosim.org/docs/fortress/ros_gz_project_template_guide)

## Communication
Gazebo→ROS2
```bash
ros2 run ros_gz_bridge parameter_bridge /TOPIC@ROS_MSG@IGN_MSG
```
>The ROS message type is followed by an @, [, or ] symbol where:
>
>- @ is a bidirectional bridge.
>- [ is a bridge from Ignition to ROS.
>- ] is a bridge from ROS to Ignition.

source: https://gazebosim.org/docs/fortress/ros2_integration#bidirectional-communication

[More info on message types and bridge communication](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md#example-1a-ignition-transport-talker-and-ros-2-listener)
> For example,
> ```bash
>ros2 run ros_gz_bridge parameter_bridge /diff_drive/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
>```
>```bash
>ros2 topic pub /diff_drive/cmd_vel geometry_msgs/Twist "linear: { x: 0.1 }"
>```

```bash
# list topics in the Ignition simulation environment
ign topic -l
# list topics in a ROS 2 system
ros2 topic list
# print messages published to a topic
ros2 topic echo /TOPIC
```