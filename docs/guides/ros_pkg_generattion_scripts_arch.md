## ROS Package Generation Scripts Architecture

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
│   ├── generate_package_xml.sh
│   ├── generate_pilz_cartesian_limits.sh
│   ├── generate_pkg_xacro.sh
│   ├── generate_robot_description_template.sh
│   ├── generate_robot_description_wrapper.sh
│   ├── generate_robots_json.sh
│   ├── generate_ros2_control_xacro.sh
│   ├── generate_rviz_config.sh
│   ├── generate_sensors_3d.sh
│   ├── generate_srdf.sh
│   ├── generate_static_robot_description.sh
│   └── robot_description_template.xacro
```

---

### Script Summaries

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

### helpers/

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

**generate_sensors_3d.sh**  
Creates sensors_3d.yaml, configuring 3D sensors for the simulation.

**generate_srdf.sh**  
Generates a Semantic Robot Description Format (SRDF) file for MoveIt, defining planning groups and collision disables for all robots.

**generate_static_robot_description.sh**  
Renders a static URDF from the generated robot_description_template.xacro using xacro or ROS 2 tools.

**robot_description_template.xacro**  
Template Xacro file used as a base for generating robot URDFs.
# ROS Package Generation Scripts Architecture

