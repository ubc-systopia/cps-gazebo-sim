#!/bin/bash
# Usage: generate_package_xml.sh <package_name> <pkg_dir>

PACKAGE_NAME="$1"
PKG_DIR="$2"

OUT_FILE="${PKG_DIR}/package.xml"

cat > "$OUT_FILE" << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>${PACKAGE_NAME}</name>
  <version>0.3.0</version>
  <description>
     An automatically generated package with all the configuration and launch files for using the mutli_arm with the MoveIt Motion Planning Framework
  </description>
  <maintainer email="sinkusroman@gmail.com">Roman Sinkus</maintainer>

  <license>BSD</license>

  <url type="website">http://moveit.ros.org/</url>
  <url type="bugtracker">https://github.com/ros-planning/moveit2/issues</url>
  <url type="repository">https://github.com/ros-planning/moveit2</url>

  <author email="sinkusroman@gmail.com">Roman Sinkus</author>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>moveit_kinematics</exec_depend>
  <exec_depend>moveit_planners</exec_depend>
  <exec_depend>moveit_simple_controller_manager</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>moveit_configs_utils</exec_depend>
  <exec_depend>moveit_ros_visualization</exec_depend>
  <exec_depend>moveit_ros_warehouse</exec_depend>
  <exec_depend>moveit_setup_assistant</exec_depend>
  <exec_depend>multi_ur_description</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>rviz_common</exec_depend>
  <exec_depend>rviz_default_plugins</exec_depend>
  <exec_depend>warehouse_ros_mongo</exec_depend>

  <export>
      <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

echo "Generated: $OUT_FILE"
