#!/bin/bash
# Usage: generate_initial_positions.sh <package_name> <pkg_dir> <robot1:pan,lift,elbow,w1,w2,w3> <robot2:...> ...
PACKAGE_NAME="$1"
PKG_DIR="$2"
shift 2

out_file="${PKG_DIR}/config/initial_positions.yaml"
echo "initial_positions:" > "$out_file"
for arg in "$@"; do
  robot_name="${arg%%:*}"
  joint_vals="${arg#*:}"
  IFS=',' read -ra vals <<< "$joint_vals"
  echo "  $robot_name:" >> "$out_file"
  echo "    ${robot_name}_shoulder_pan_joint: ${vals[0]}" >> "$out_file"
  echo "    ${robot_name}_shoulder_lift_joint: ${vals[1]}" >> "$out_file"
  echo "    ${robot_name}_elbow_joint: ${vals[2]}" >> "$out_file"
  echo "    ${robot_name}_wrist_1_joint: ${vals[3]}" >> "$out_file"
  echo "    ${robot_name}_wrist_2_joint: ${vals[4]}" >> "$out_file"
  echo "    ${robot_name}_wrist_3_joint: ${vals[5]}" >> "$out_file"
done
