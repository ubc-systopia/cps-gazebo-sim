#!/bin/bash
# Usage: generate_pilz_cartesian_limits.sh <package_name> <pkg_dir>

PACKAGE_NAME="$1"
PKG_DIR="$2"

OUT_FILE="${PKG_DIR}/config/pilz_cartesian_limits.yaml"
mkdir -p "${PKG_DIR}/config"

# Write YAML using echo (spaces only, no tabs)
echo "# Limits for the Pilz planner" >  "$OUT_FILE"
echo "cartesian_limits:"              >> "$OUT_FILE"
echo "  max_trans_vel: 1.0"          >> "$OUT_FILE"
echo "  max_trans_acc: 2.25"         >> "$OUT_FILE"
echo "  max_trans_dec: -5.0"         >> "$OUT_FILE"
echo "  max_rot_vel: 1.57"           >> "$OUT_FILE"

echo "Generated Pilz cartesian limits: $OUT_FILE"
