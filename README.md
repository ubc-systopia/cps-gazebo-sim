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

The stack this project is built and run against:
- [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Gazebo Fortress (Ignition)](https://gazebosim.org/docs/fortress/install_ubuntu) + the [`ros-gz`](https://github.com/gazebosim/ros_gz) bridge
- [MoveIt 2](https://moveit.ros.org/install-moveit2/binary/), `ros2_control` and `gz_ros2_control`

## Installation
From the repository root, run:

```bash
scripts/setup.sh
```

This single script is idempotent and brings a fresh Ubuntu 22.04 machine to the
exact environment used here: it installs ROS 2 Humble (desktop + dev tools),
Gazebo Fortress, `ros-gz`, MoveIt 2, the `ros2_control`/`gz_ros2_control`
controllers, the Python tooling the `scripts/` commands need, resolves all
workspace dependencies with `rosdep`, and runs a full `colcon build`. Re-run it
any time after pulling new changes.

When it finishes, source ROS and the local overlay in each new shell (add the
first line to your `~/.bashrc` for convenience):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Contributing (dev tooling)
If you plan to contribute, install the pre-commit hooks (lint/format) that run on
each commit:

```bash
pip install -r requirements.dev.txt
pre-commit install
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
