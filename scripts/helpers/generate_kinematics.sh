#!/bin/bash
# Usage: generate_kinematics.sh <package_name> <pkg_dir> <robot1> <robot2> ...
# Emits one kinematics group per arm using the pick_ik solver (global mode).
# pick_ik is joint-limit-aware and avoids the wound "pretzel" solutions KDL
# returns for 6-DOF arms with orientation goals.

PACKAGE_NAME="$1"
PKG_DIR="$2"
shift 2

out_file="${PKG_DIR}/config/kinematics.yaml"
mkdir -p "${PKG_DIR}/config"
: > "$out_file"   # truncate so re-running doesn't append duplicates

for robot in "$@"; do
  cat >> "$out_file" << EOF
${robot}:
  kinematics_solver: pick_ik/PickIkPlugin
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3
  mode: global
  position_scale: 1.0
  rotation_scale: 0.5
  position_threshold: 0.001
  orientation_threshold: 0.01
  cost_threshold: 0.001
  minimal_displacement_weight: 0.0
  gd_step_size: 0.0001
EOF
done
