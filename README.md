# cps-gazebo-sim

## System Requirements
- Ubuntu Jammy 22.04
- CMake version > 3.5

## Installation
[ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
> For convenience, add the following line to *.bashrc*. Other possible values are setup.sh, setup.zsh
>```
>source /opt/ros/humble/setup.bash
>```

[Gazebo Fortress](https://gazebosim.org/docs/fortress/install_ubuntu)
or by default as the recommended pairing for ROS2 Humble via the command:

```
sudo apt-get install ros-humble-ros-gz
```

### Drivers for Robots
|Robot|Installation Link|
|---|---|
|ViperX 300s|https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html|
|UR3e series|https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble|

## To run
https://gazebosim.org/docs/all/getstarted

```ign gazebo shapes.sdf```

Build
```cd ~/cps-gazebo-sim
colcon build --cmake-args -DBUILD_TESTING=ON
```
Run
```
# source the workspace
. ~/cps-gazebo-sim/install/setup.sh
# launch the simulation and visualize in RViz
ros2 launch ros_gz_example_bringup diff_drive.launch.py
```

## All the technical steps that created this project
1. [Cloning a template](https://gazebosim.org/docs/fortress/ros_gz_project_template_guide)
2. 