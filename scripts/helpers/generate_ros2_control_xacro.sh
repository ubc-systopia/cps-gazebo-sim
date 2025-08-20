#!/bin/bash
# Usage: generate_ros2_control_xacro.sh <package_name> <pkg_dir> <robot1> [<robot2> ...]
# Generates <pkg_dir>/urdf/<package_name>.ros2_control.xacro emulating the two_arm multi_arm.ros2_control.xacro

set -euo pipefail

PACKAGE_NAME="$1"
PKG_DIR="$2"
shift 2 || true

if [ "$#" -eq 0 ]; then
  echo "[ERROR] No robot names provided to generate_ros2_control_xacro.sh" >&2
  exit 1
fi

OUT_FILE="${PKG_DIR}/urdf/${PACKAGE_NAME}.ros2_control.xacro"
mkdir -p "${PKG_DIR}/urdf"

echo "[INFO] Generating ros2_control XACRO: ${OUT_FILE}"

# Header and macro opening
cat > "${OUT_FILE}" << 'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="multi_arm_ros2_control" params="name initial_positions_file">
    <xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)['initial_positions']}"/>

    <ros2_control name="${name}" type="system">
      <hardware>
        <plugin>ign_ros2_control/IgnitionSystem</plugin>
      </hardware>
EOF

# Per-robot joint blocks (correctly escaped xacro property references)
for robot in "$@"; do
  {
    echo "      <!-- ${robot} joints -->"
    echo "      <joint name=\"${robot}_shoulder_pan_joint\">"
    echo "        <command_interface name=\"position\"/>"
    echo "        <state_interface name=\"position\">"
    echo "          <param name=\"initial_value\">\${initial_positions['${robot}_shoulder_pan_joint']}</param>"
    echo "        </state_interface>"
    echo "        <state_interface name=\"velocity\"/>"
    echo "      </joint>"

    echo "      <joint name=\"${robot}_shoulder_lift_joint\">"
    echo "        <command_interface name=\"position\"/>"
    echo "        <state_interface name=\"position\">"
    echo "          <param name=\"initial_value\">\${initial_positions['${robot}_shoulder_lift_joint']}</param>"
    echo "        </state_interface>"
    echo "        <state_interface name=\"velocity\"/>"
    echo "      </joint>"

    echo "      <joint name=\"${robot}_elbow_joint\">"
    echo "        <command_interface name=\"position\"/>"
    echo "        <state_interface name=\"position\">"
    echo "          <param name=\"initial_value\">\${initial_positions['${robot}_elbow_joint']}</param>"
    echo "        </state_interface>"
    echo "        <state_interface name=\"velocity\"/>"
    echo "      </joint>"

    echo "      <joint name=\"${robot}_wrist_1_joint\">"
    echo "        <command_interface name=\"position\"/>"
    echo "        <state_interface name=\"position\">"
    echo "          <param name=\"initial_value\">\${initial_positions['${robot}_wrist_1_joint']}</param>"
    echo "        </state_interface>"
    echo "        <state_interface name=\"velocity\"/>"
    echo "      </joint>"

    echo "      <joint name=\"${robot}_wrist_2_joint\">"
    echo "        <command_interface name=\"position\"/>"
    echo "        <state_interface name=\"position\">"
    echo "          <param name=\"initial_value\">\${initial_positions['${robot}_wrist_2_joint']}</param>"
    echo "        </state_interface>"
    echo "        <state_interface name=\"velocity\"/>"
    echo "      </joint>"

    echo "      <joint name=\"${robot}_wrist_3_joint\">"
    echo "        <command_interface name=\"position\"/>"
    echo "        <state_interface name=\"position\">"
    echo "          <param name=\"initial_value\">\${initial_positions['${robot}_wrist_3_joint']}</param>"
    echo "        </state_interface>"
    echo "        <state_interface name=\"velocity\"/>"
    echo "      </joint>"
  } >> "${OUT_FILE}"
done

# Close macro and robot
cat >> "${OUT_FILE}" << 'EOF'
    </ros2_control>
  </xacro:macro>
</robot>
EOF

echo "[SUCCESS] Generated ros2_control XACRO: ${OUT_FILE}"
