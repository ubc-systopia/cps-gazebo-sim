#!/bin/bash
# Usage: generate_robots_json.sh <package_name> <pkg_dir> <robotSpec1> [<robotSpec2> ...]
# robotSpec format: <key>:<model>:<x>:<y>:<z>:<R>:<P>:<Y>:<shoulder_pan>:<shoulder_lift>:<elbow>:<wrist_1>:<wrist_2>:<wrist_3>

set -euo pipefail

PACKAGE_NAME="$1"
PKG_DIR="$2"
shift 2

OUT_FILE="${PKG_DIR}/config/robots.json"
mkdir -p "${PKG_DIR}/config"

if [ "$#" -lt 1 ]; then
  echo "No robot specs provided to generate_robots_json.sh" >&2
  exit 1
fi

{
  echo "{";
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

echo "Generated robots.json: $OUT_FILE"
