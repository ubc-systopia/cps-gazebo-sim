#!/bin/bash
# Usage: generate_robot_description_template.sh <package_name> <pkg_dir> <robotSpec1> [<robotSpec2> ...]
# robotSpec format: key:model:x:y:z:R:P:Y:pan:lift:elbow:w1:w2:w3

set -euo pipefail

PACKAGE_NAME="$1"
PKG_DIR="$2"
shift 2 || true

OUT_FILE="${PKG_DIR}/urdf/robot_description_template.xacro"
mkdir -p "${PKG_DIR}/urdf"

if [ "$#" -eq 0 ]; then
  echo "No robot specs provided to generate_robot_description_template.sh" >&2
  exit 1
fi

# Begin file header
{
  echo "<?xml version=\"1.0\"?>"
  echo "<robot xmlns:xacro=\"http://wiki.ros.org/xacro\" name=\"multi_arm\">"
  echo "  <!-- declare per-robot arguments (defaults set from user inputs) -->"
} > "$OUT_FILE"

idx=0
for spec in "$@"; do
  IFS=":" read -r key model x y z R P Y pan lift elbow w1 w2 w3 <<< "$spec"
  idx=$((idx+1))
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

# Per-robot ros2_control joints with initial positions
for spec in "$@"; do
  IFS=":" read -r key model x y z R P Y pan lift elbow w1 w2 w3 <<< "$spec"
  {
    echo "    <!-- ${key} joints -->"
    echo "    <joint name=\"${key}_shoulder_pan_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\">"
    echo "        <param name=\"initial_value\">${pan}</param>"
    echo "      </state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
    echo "    <joint name=\"${key}_shoulder_lift_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\">"
    echo "        <param name=\"initial_value\">${lift}</param>"
    echo "      </state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
    echo "    <joint name=\"${key}_elbow_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\">"
    echo "        <param name=\"initial_value\">${elbow}</param>"
    echo "      </state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
    echo "    <joint name=\"${key}_wrist_1_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\">"
    echo "        <param name=\"initial_value\">${w1}</param>"
    echo "      </state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
    echo "    <joint name=\"${key}_wrist_2_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\">"
    echo "        <param name=\"initial_value\">${w2}</param>"
    echo "      </state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
    echo "    <joint name=\"${key}_wrist_3_joint\">"
    echo "      <command_interface name=\"position\"/>"
    echo "      <command_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"position\">"
    echo "        <param name=\"initial_value\">${w3}</param>"
    echo "      </state_interface>"
    echo "      <state_interface name=\"velocity\"/>"
    echo "      <state_interface name=\"effort\"/>"
    echo "    </joint>"
  } >> "$OUT_FILE"
done

# Close ros2_control block
echo "  </ros2_control>" >> "$OUT_FILE"

# Per-robot kinematic chains (no ros2_control tags inside)
for spec in "$@"; do
  IFS=":" read -r key model x y z R P Y pan lift elbow w1 w2 w3 <<< "$spec"
  {
    echo ""
    echo "  <!-- ${key} kinematic chain WITHOUT ros2_control (sim_ignition true for visuals) -->"
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

# Ignition ros2_control plugin referencing the package controllers.yaml
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

echo "Generated robot_description_template.xacro: $OUT_FILE"
