#!/bin/bash
# Usage: generate_robots_json.sh <package_name> <pkg_dir> <launch_rviz> <robotSpec1> [<robotSpec2> ...]
#   launch_rviz: "true" or "false" - whether the generated demo.launch.py should
#                start RViz (read at launch time from robots.json).
#   robotSpec  : <key>:<model>:<x>:<y>:<z>:<R>:<P>:<Y>:<shoulder_pan>:<shoulder_lift>:<elbow>:<wrist_1>:<wrist_2>:<wrist_3>

set -euo pipefail

PACKAGE_NAME="$1"
PKG_DIR="$2"
LAUNCH_RVIZ="$3"
shift 3

# normalise to bare JSON true/false
case "${LAUNCH_RVIZ,,}" in
  true|yes|y|1)  LAUNCH_RVIZ_JSON="true" ;;
  false|no|n|0)  LAUNCH_RVIZ_JSON="false" ;;
  *) echo "launch_rviz must be true/false (got: ${LAUNCH_RVIZ})" >&2; exit 1 ;;
esac

OUT_FILE="${PKG_DIR}/config/robots.json"
mkdir -p "${PKG_DIR}/config"

if [ "$#" -lt 1 ]; then
  echo "No robot specs provided to generate_robots_json.sh" >&2
  exit 1
fi

{
  echo "{";
  echo "  \"launch_rviz\": ${LAUNCH_RVIZ_JSON},";
  echo "  \"robot_arms\": [";

  first=1
  for spec in "$@"; do
    IFS=':' read -r key model x y z R P Y pan lift elbow w1 w2 w3 <<< "$spec"
    if [ $first -eq 0 ]; then echo ","; fi
    first=0
    cat << EOF
    {
      "key": "${key}",
      "model": "${model}",
      "base_coordinates": {
        "x": "${x}",
        "y": "${y}",
        "z": "${z}",
        "R": "${R}",
        "P": "${P}",
        "Y": "${Y}"
      },
      "initial_joints": {
        "shoulder_pan": "${pan}",
        "shoulder_lift": "${lift}",
        "elbow": "${elbow}",
        "wrist_1": "${w1}",
        "wrist_2": "${w2}",
        "wrist_3": "${w3}"
      }
    }
EOF
  done

  echo "  ]";
  echo "}";
} > "$OUT_FILE"

chmod +x "$OUT_FILE" 2>/dev/null || true

echo "Generated robots.json: $OUT_FILE (launch_rviz=${LAUNCH_RVIZ_JSON})"
