#!/bin/bash
# Usage: generate_workflow_demo.sh <package_name> <pkg_dir>
# Emits a thin demo workflow script that drives the package via the shared
# multi_arm_control library. All control logic lives in the library; this script
# is just task choreography, and discovers the arms from the package's robots.json.

PACKAGE_NAME="$1"
PKG_DIR="$2"

mkdir -p "${PKG_DIR}/scripts"
OUT_FILE="${PKG_DIR}/scripts/workflow_demo.py"

cat > "$OUT_FILE" << PY
#!/usr/bin/env python3
"""Demo workflow for ${PACKAGE_NAME}, built on the shared multi_arm_control API.

Run WHILE the package is up (Gazebo + move_group):

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/${PACKAGE_NAME}/scripts/workflow_demo.py

All control logic lives in the multi_arm_control library; this script is just the
task choreography. ArmFleet discovers the arms from this package's robots.json.
"""

import rclpy

from multi_arm_control import ArmFleet, HOME_JOINTS, UPRIGHT_JOINTS

PACKAGE = "${PACKAGE_NAME}"


def main():
    rclpy.init()
    fleet = ArmFleet(PACKAGE)
    try:
        names = fleet.arm_names

        # 1. All arms up together, then down together - one coordinated,
        #    collision-checked motion (needs the all_arms SRDF group).
        if len(names) >= 2:
            fleet.move_coordinated({n: UPRIGHT_JOINTS for n in names})
            fleet.move_coordinated({n: HOME_JOINTS for n in names})

        # 2. Then each arm on its own, in sequence: up then down.
        for n in names:
            fleet.arm(n).upright()
            fleet.arm(n).home()
    finally:
        fleet.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
PY

chmod +x "$OUT_FILE"
echo "Generated workflow demo: $OUT_FILE"
