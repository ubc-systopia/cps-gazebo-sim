# Software dependencies and versions
Generic information and helpful links related to different releases of the software used in the CPS Multi Robot Gazebo Sim project.

## Current Stack
- Gazebo Sim (Fortress)
- ROS2 Humble
- MoveIt2

## Simulator
Gazebo has two versions: "Gazebo Classic" and "Gazebo Sim," the latter of which was previously named "Ignition."

Gazebo Classic will reach its end-of-life (EOL) in January 2025. However, some open-source packages, such as those for ROS manipulators provided by Interbotix, still use it as the default simulator. Consequently, we sometimes need to manually migrate Gazebo Classic ROS2 packages to Gazebo Sim. These modified packages need to be built manually and made accessible as shared ROS2 packages. See [Migrating Gazebo Classic ROS2 Packages](https://github.com/gazebosim/docs/blob/master/migrating_gazebo_classic_ros2_packages.md).

Gazebo Sim represents the modern series of simulators, with multiple named releases as of 2022. The latest LTS (Long-Term Support) release is Gazebo Harmonic, which is recommended for use with ROS2 versions newer than Humble. **Currently, we use Gazebo Fortress (LTS), which will reach EOL in September 2026.** See [Gazebo Sim Get Started](https://gazebosim.org/docs/fortress/getstarted/).

## Robot Operating System
ROS facilitates robot software development and is used to provide a level of abstraction for hardware interfaces. It standardizes communication between different components of a robot through topics, services, and actions. ROS2 is an upgrade from ROS1, offering enhanced features and improved performance. The last release of ROS1 will reach EOL in May 2025, while the **ROS2 Humble distribution will reach EOL in May 2027**. ROS is well-documented. See [How-to Guides](https://docs.ros.org/en/humble/How-To-Guides.html). Note that the Niryo NED2 is not currently compatible with ROS2. See [Migrating from ROS1 to ROS2](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html).

## Control
