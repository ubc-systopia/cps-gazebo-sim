#!/bin/bash
# Usage: generate_pkg_xacro.sh <package_name> <pkg_dir> <robotSpec1> [<robotSpec2> ...]
# robotSpec format: key:model:x:y:z:R:P:Y:pan:lift:elbow:w1:w2:w3
# Generates <pkg_dir>/urdf/<package_name>.urdf.xacro

set -euo pipefail

PACKAGE_NAME="$1"
PKG_DIR="$2"
shift 2 || true

if [ "$#" -eq 0 ]; then
  echo "[ERROR] No robot specs provided to generate_pkg_xacro.sh" >&2
  exit 1
fi

OUT_FILE="${PKG_DIR}/urdf/${PACKAGE_NAME}.urdf.xacro"
mkdir -p "${PKG_DIR}/urdf"

# Header and per-robot args
{
  echo "<?xml version=\"1.0\"?>"
  echo "<robot xmlns:xacro=\"http://wiki.ros.org/xacro\" name=\"multi_arm\">"
  echo "  <!-- declare arguments -->"
} > "$OUT_FILE"

for spec in "$@"; do
  IFS=":" read -r key model x y z R P Y pan lift elbow w1 w2 w3 <<< "$spec"
  {
    echo "  <xacro:arg name=\"${key}_ur_type\" default=\"${model}\"/>"
    echo "  <xacro:arg name=\"${key}_x\" default=\"${x}\"/>"
    echo "  <xacro:arg name=\"${key}_y\" default=\"${y}\"/>"
    echo "  <xacro:arg name=\"${key}_z\" default=\"${z}\"/>"
    echo "  <xacro:arg name=\"${key}_R\" default=\"${R}\"/>"
    echo "  <xacro:arg name=\"${key}_P\" default=\"${P}\"/>"
    echo "  <xacro:arg name=\"${key}_Y\" default=\"${Y}\"/>"
  } >> "$OUT_FILE"
done

{
  echo ""
  echo "  <!-- world reference frame -->"
  echo "  <link name=\"world\"/>"
  echo ""
  echo "  <!-- include the core URDF macro definitions -->"
  echo "  <xacro:include filename=\"\$(find multi_ur_description)/urdf/ur_macro.xacro\"/>"
  echo ""
  echo "  <!-- Unified ros2_control for all arms (Ignition simulation) -->"
  echo "  <ros2_control name=\"multi_arm_controller_manager\" type=\"system\">"
  echo "    <hardware>"
  echo "      <plugin>ign_ros2_control/IgnitionSystem</plugin>"
  echo "    </hardware>"
} >> "$OUT_FILE"

# ros2_control joints per robot
for spec in "$@"; do
  IFS=":" read -r key model x y z R P Y pan lift elbow w1 w2 w3 <<< "$spec"
  {
    echo "    <!-- ${key} joints -->"
    echo "    <joint name=\"${key}_shoulder_pan_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\"><param name=\"initial_value\">${pan}</param></state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
    echo "    <joint name=\"${key}_shoulder_lift_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\"><param name=\"initial_value\">${lift}</param></state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
    echo "    <joint name=\"${key}_elbow_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\"><param name=\"initial_value\">${elbow}</param></state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
    echo "    <joint name=\"${key}_wrist_1_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\"><param name=\"initial_value\">${w1}</param></state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
    echo "    <joint name=\"${key}_wrist_2_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\"><param name=\"initial_value\">${w2}</param></state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
    echo "    <joint name=\"${key}_wrist_3_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\"><param name=\"initial_value\">${w3}</param></state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
  } >> "$OUT_FILE"
done

echo "  </ros2_control>" >> "$OUT_FILE"

# Per-robot ur_robot instances
for spec in "$@"; do
  IFS=":" read -r key model x y z R P Y pan lift elbow w1 w2 w3 <<< "$spec"
  {
    echo ""
    echo "  <!-- ${key} kinematic chain WITHOUT internal ros2_control -->"
    echo "  <xacro:ur_robot"
    echo "    name=\"${key}\""
    echo "    tf_prefix=\"${key}_\""
    echo "    parent=\"world\""
    echo "    joint_limits_parameters_file=\"\$(find multi_ur_description)/config/\$(arg ${key}_ur_type)/joint_limits.yaml\""
    echo "    kinematics_parameters_file=\"\$(find multi_ur_description)/config/\$(arg ${key}_ur_type)/default_kinematics.yaml\""
    echo "    physical_parameters_file=\"\$(find multi_ur_description)/config/\$(arg ${key}_ur_type)/physical_parameters.yaml\""
    echo "    visual_parameters_file=\"\$(find multi_ur_description)/config/\$(arg ${key}_ur_type)/visual_parameters.yaml\""
    echo "    generate_ros2_control_tag=\"false\""
    echo "    sim_gazebo=\"false\""
    echo "    sim_ignition=\"true\">"
    echo "    <origin xyz=\"\$(arg ${key}_x) \$(arg ${key}_y) \$(arg ${key}_z)\" rpy=\"\$(arg ${key}_R) \$(arg ${key}_P) \$(arg ${key}_Y)\"/>"
    echo "  </xacro:ur_robot>"
  } >> "$OUT_FILE"
done


# Debug: print PACKAGE_NAME to ensure it's set
if [ -z "$PACKAGE_NAME" ]; then
  echo "[ERROR] PACKAGE_NAME is empty when writing Gazebo plugin block!" >&2
  exit 2
else
  echo "[INFO] PACKAGE_NAME for Gazebo plugin: $PACKAGE_NAME" >&2
fi

{
  echo ""
  echo "  <gazebo>"
  echo "    <plugin filename=\"libign_ros2_control-system.so\" name=\"ign_ros2_control::IgnitionROS2ControlPlugin\">"
  echo "      <parameters>\$(find ${PACKAGE_NAME})/config/controllers.yaml</parameters>"
  echo "    </plugin>"
  echo "  </gazebo>"
  echo ""
  echo "</robot>"
} >> "$OUT_FILE"

echo "Generated pkg XACRO: $OUT_FILE"
