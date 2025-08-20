#!/bin/bash
# Usage: generate_moveit_controllers.sh <package_name> <pkg_dir> <controller1:j1,j2,...> [<controller2:j1,j2,...> ...]

PACKAGE_NAME="$1"
PKG_DIR="$2"
shift 2

OUT_FILE="${PKG_DIR}/config/moveit_controllers.yaml"

# Header: MoveIt simple controller manager
echo "moveit_simple_controller_manager:" > "$OUT_FILE"
echo "  controller_names:" >> "$OUT_FILE"

# List all controller names
for arg in "$@"; do
  ctrl_name="${arg%%:*}"
  echo "    - ${ctrl_name}" >> "$OUT_FILE"
done

# Define each controller block
for arg in "$@"; do
  ctrl_name="${arg%%:*}"
  joint_list="${arg#*:}"
  echo "  ${ctrl_name}:" >> "$OUT_FILE"
  echo "    type: FollowJointTrajectory" >> "$OUT_FILE"
  echo "    action_ns: follow_joint_trajectory" >> "$OUT_FILE"
  echo "    default: true" >> "$OUT_FILE"
  echo "    joints:" >> "$OUT_FILE"
  IFS=',' read -ra joints <<< "$joint_list"
  for j in "${joints[@]}"; do
    echo "      - $j" >> "$OUT_FILE"
  done
done

