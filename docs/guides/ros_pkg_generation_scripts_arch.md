# ROS Package Generation Scripts Architecture

This file outlines the structure and purpose of each script used for automating ROS package generation in the workspace.

```
scripts/
├── create-ros-pkg
├── integration-tests
├── launch_PKG_NAME
├── multiple-robots-moveit
├── multiple-robots-with-controllers-no-moveit
├── helpers/
│   ├── generate_cmakelists.sh
│   ├── generate_controllers.sh
│   ├── generate_demo_launch.sh
│   ├── generate_initial_positions.sh
│   ├── generate_joint_limits.sh
│   ├── generate_kinematics.sh
│   ├── generate_launch_script.sh
│   ├── generate_moveit_controllers.sh
│   ├── generate_ompl_planning.sh
│   ├── generate_package_xml.sh
│   ├── generate_pilz_cartesian_limits.sh
│   ├── generate_pkg_xacro.sh
│   ├── generate_robot_description_template.sh
│   ├── generate_robot_description_wrapper.sh
│   ├── generate_robots_json.sh
│   ├── generate_ros2_control_xacro.sh
│   ├── generate_rviz_config.sh
│   ├── generate_scene_publisher.sh
│   ├── generate_sensors_3d.sh
│   ├── generate_srdf.sh
│   ├── generate_static_robot_description.sh
│   ├── generate_workflow_demo.sh
│   └── generate_world_sdf.sh
```

---

## Script Summaries

**create-ros-pkg**  
Main entrypoint for package generation. Interactively collects robot configuration (number, type, positions, orientations, joint values) and calls helper scripts to generate all necessary files for a new ROS package supporting n robots.

**integration-tests**  
Runs integration tests for the workspace using pytest, sourcing ROS and workspace environments.

**launch_PKG_NAME**  
Builds the workspace and launches the demo for the generated package using ROS 2 launch.

**multiple-robots-moveit**  
Launches a MoveIt-enabled demo for multiple robots, sourcing the workspace and ROS environment.

**multiple-robots-with-controllers-no-moveit**  
Launches a demo for multiple robots with controllers, but without MoveIt, sourcing the workspace and ROS environment.

## helpers/

**generate_cmakelists.sh**  
Generates a CMakeLists.txt for the package, including installation rules for launch, config, and urdf folders.

**generate_controllers.sh**  
Creates a controllers.yaml file defining joint trajectory controllers and joint state broadcasters for all robots.

**generate_demo_launch.sh**  
Generates a ROS 2 Python launch file (`demo.launch.py`) that loads robot configurations and starts required nodes.

**generate_initial_positions.sh**  
Creates initial_positions.yaml, specifying initial joint values for each robot.

**generate_joint_limits.sh**  
Creates joint_limits.yaml, setting velocity and acceleration limits for each robot's joints.

**generate_kinematics.sh**  
Creates kinematics.yaml, configuring kinematic solvers for each robot.

**generate_launch_script.sh**  
Generates a launch script in the scripts/ folder to build and launch the new package.

**generate_moveit_controllers.sh**  
Creates moveit_controllers.yaml, mapping each robot's controller to the MoveIt FollowJointTrajectory interface.

**generate_ompl_planning.sh**  
Creates ompl_planning.yaml with one planning-group block per arm (plus an `all_arms` group when there are two or more), matching the SRDF groups. Uses a finer `longest_valid_segment_fraction` than MoveIt's default so path simplification can't shortcut a link through a static collision object; `planner_configs` is omitted so MoveIt merges in its default planners (RRTConnect, …).

**generate_package_xml.sh**  
Generates package.xml, listing dependencies and metadata for the new package.

**generate_pilz_cartesian_limits.sh**  
Creates pilz_cartesian_limits.yaml, setting Cartesian limits for the Pilz planner.

**generate_pkg_xacro.sh**  
Generates the main URDF Xacro file for all robots, including arguments for robot types, positions, and joint values.

**generate_robot_description_template.sh**  
Creates a template Xacro file for robot description, parameterized for n robots.

**generate_robot_description_wrapper.sh**  
Generates a wrapper Xacro file that imports the main URDF and ros2_control macro for flexible instantiation.

**generate_robots_json.sh**  
Creates robots.json, describing all robot arms, their models, positions, and initial joint values.

**generate_ros2_control_xacro.sh**  
Generates a ros2_control Xacro macro for all robots, loading initial positions and specifying hardware plugins.

**generate_rviz_config.sh**  
Creates an RViz configuration file for visualizing all robots in the simulation.

**generate_scene_publisher.sh**  
Emits `scripts/scene_publisher.py`, a launch-time node that reads the package's `config/robots.json` `static_objects` and adds them to the MoveIt planning scene (via `/apply_planning_scene`) as collision objects, so the arms plan around them. Installed to `lib/<pkg>` and launched (delayed) by `demo.launch.py` when static objects are present.

**generate_sensors_3d.sh**  
Creates sensors_3d.yaml, configuring 3D sensors for the simulation.

**generate_srdf.sh**  
Generates a Semantic Robot Description Format (SRDF) file for MoveIt, defining planning groups and collision disables for all robots.

**generate_static_robot_description.sh**  
Renders a static URDF from the generated robot_description_template.xacro using xacro or ROS 2 tools.

**generate_workflow_demo.sh**  
Emits a thin `scripts/workflow_demo.py` that drives the package via the shared `multi_arm_control` library. All control logic lives in the library; this file is just task choreography and discovers the arms from the package's `robots.json`.

**generate_world_sdf.sh**  
Reads `config/robots.json` and, if it has a non-empty `static_objects` array, renders `config/world.sdf` (the standard empty world plus one static `<model>` per object). If there are no static objects, any stale `world.sdf` is removed so the package falls back to launching `empty.sdf`.

