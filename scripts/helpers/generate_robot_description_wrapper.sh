#!/bin/bash
# Usage: generate_robot_description_wrapper.sh <package_name> <pkg_dir>
# Generates <pkg_dir>/urdf/<package_name>_wrapper.urdf.xacro

set -euo pipefail

PACKAGE_NAME="$1"
PKG_DIR="$2"

OUT_FILE="${PKG_DIR}/urdf/${PACKAGE_NAME}_wrapper.urdf.xacro"
mkdir -p "${PKG_DIR}/urdf"

echo "[INFO] Generating wrapper XACRO: ${OUT_FILE}"

cat > "${OUT_FILE}" << EOF
<?xml version="1.0"?>
<!-- Wrapper modeled after two_arm mutli_arm.urdf.xacro, generalized for any number of robots -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="${PACKAGE_NAME}">
  <!-- Initial joint positions YAML used by the ros2_control macro -->
  <xacro:arg name="initial_positions_file" default="\$(find ${PACKAGE_NAME})/config/initial_positions.yaml" />

  <!-- Import multi-robot URDF assembly for this package -->
  <xacro:include filename="\$(find ${PACKAGE_NAME})/urdf/${PACKAGE_NAME}.urdf" />

  <!-- Import ros2_control macro Xacro placed alongside this wrapper in urdf/ -->
  <xacro:include filename="${PACKAGE_NAME}.ros2_control.xacro" />

  <!-- Optionally instantiate the control macro (kept commented like the reference): -->
  <!-- <xacro:multi_arm_ros2_control name="ign_ros2_control/IgnitionSystem" initial_positions_file="\$(arg initial_positions_file)"/> -->

</robot>
EOF

echo "[SUCCESS] Generated wrapper XACRO: ${OUT_FILE}"
