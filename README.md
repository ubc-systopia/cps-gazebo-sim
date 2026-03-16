# cps-gazebo-sim

## Included packages

* `multi_arm_lab_sim_description` - holds the sdf description of the simulated system and any other assets.

* `multi_arm_lab_sim_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `multi_arm_lab_sim_application` - holds ros2 specific code and configurations.

* `multi_arm_lab_sim_bringup` - holds launch files and high level utilities.

## Requirements
- Ubuntu Jammy 22.04
- CMake version >= 3.5
- Python 3.10

## Installation
1. Navigate to the root directory
2. Run `scripts/setup.sh`
---

1. [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
    > For convenience, add the following line to *.bashrc*. Other possible values are setup.sh, setup.zsh
    >```bash
    >source /opt/ros/humble/setup.bash
    >```

2. Dependencies:
    ```bash
    cd ~/cps-gazebo-sim
    source /opt/ros/humble/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro humble
    ```

3. [Gazebo Fortress](https://gazebosim.org/docs/fortress/install_ubuntu)
4. `ros-gz` package which includes the necessary bridges and interfaces between ROS2 Humble and Gazebo Fortress
    ```bash
    sudo apt-get install ros-humble-ros-gz
    ```
5. Install Interbotix robot packages
https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html#amd64-architecture
(instead of 6. maybe edit the github source links for repos that I modified. in the xsarm_amd64_install.sh script)
6. Update sources by running (*insert my script which updates remote urls to my customized repos*)

7. [MoveIt2](https://moveit.ros.org/install-moveit2/binary/)
```bash
sudo apt install ros-humble-moveit
```

## Documentation
Documentation for this repo is generated with [Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html). To update documentation: `cd docs`, make necessary edits, then run `make html`. View the output `_build/html/index.html` in a web browser.

Presentations:
- [Presentation](https://docs.google.com/presentation/d/1053tHjzkwP5x19ikkgmAjDZcN7jM4G29i3L29Vi7hdI/edit?usp=sharing) from Summer 2024 Intern/UG Event.
- [Presentation](https://www.canva.com/design/DAGwwh0NeGw/VnevSo_6Vuk2WIOzhP9oVQ/edit?utm_content=DAGwwh0NeGw&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton) from Summer 2025 Intern/UG Event.

## Sources
- [Gazebo fortress docs](https://gazebosim.org/docs/all/getstarted)
- [Gazebo Sim 6.16 API Reference](https://gazebosim.org/api/sim/6/)
- [ros_gz_project_template](https://gazebosim.org/docs/fortress/ros_gz_project_template_guide)
- [UR3e description](https://github.com/ros-industrial/universal_robot/tree/noetic-devel/ur_description/meshes/ur3e)
- [SDFormat](http://sdformat.org/spec?elem=sdf&ver=1.8)
- [Humble Testing docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)

## Gotchas
- ufw firewall must be disabled for Gazebo to run
