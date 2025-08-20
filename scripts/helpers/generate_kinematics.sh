#!/bin/bash
# Usage: generate_kinematics.sh <package_name> <pkg_dir> <robot1> <robot2> ...

PACKAGE_NAME="$1"
PKG_DIR="$2"
shift 2

out_file="${PKG_DIR}/config/kinematics.yaml"
for robot in "$@"; do
  echo "$robot:" >> "$out_file"
  echo "  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin" >> "$out_file"
  echo "  kinematics_solver_search_resolution: 0.005" >> "$out_file"
  echo "  kinematics_solver_timeout: 0.005" >> "$out_file"
  echo "  kinematics_solver_attempts: 3" >> "$out_file"
done
