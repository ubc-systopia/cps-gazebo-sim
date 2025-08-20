#!/bin/bash
# Usage: generate_static_robot_description.sh <package_name> <pkg_dir>
# Creates a static URDF by executing the generated robot_description_template.xacro

set -euo pipefail

PACKAGE_NAME="$1"
PKG_DIR="$2"

SRC_XACRO="${PKG_DIR}/urdf/robot_description_template.xacro"
OUT_URDF="${PKG_DIR}/urdf/robot_description_template.urdf"

if [ ! -f "${SRC_XACRO}" ]; then
  echo "[ERROR] Xacro template not found: ${SRC_XACRO}" >&2
  exit 1
fi

# Ensure output directory exists
mkdir -p "${PKG_DIR}/urdf"

echo "[INFO] Rendering static URDF from: ${SRC_XACRO}"

status=0

# Try xacro CLI first, then ros2, then python -m xacro
if command -v xacro >/dev/null 2>&1; then
  xacro "${SRC_XACRO}" -o "${OUT_URDF}" || status=$?
elif command -v ros2 >/dev/null 2>&1; then
  ros2 run xacro xacro "${SRC_XACRO}" -o "${OUT_URDF}" || status=$?
else
  if python3 - << 'PY'
import sys
try:
    import xacro  # noqa: F401
except Exception:
    sys.exit(1)
PY
  then
    python3 -m xacro "${SRC_XACRO}" -o "${OUT_URDF}" || status=$?
  else
    echo "[ERROR] Could not find xacro CLI. Please ensure the ROS xacro package is installed." >&2
    status=127
  fi
fi

if [ $status -ne 0 ]; then
  echo "[WARNING] Static URDF generation failed (exit=${status}). Continuing. You can render later after sourcing ROS (so multi_ur_description is discoverable)." >&2
else
  echo "[SUCCESS] Static URDF generated: ${OUT_URDF}"
fi
