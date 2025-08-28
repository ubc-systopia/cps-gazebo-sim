#!/bin/bash
# Usage: generate_launch_script.sh <package_name> <pkg_dir>
# Creates a launch script in the scripts/ folder for the new package

PACKAGE_NAME="$1"
PKG_DIR="$2"
SCRIPTS_DIR="$(dirname "$0")/../.."
LAUNCH_SCRIPT="${SCRIPTS_DIR}/scripts/launch_${PACKAGE_NAME}"

cat > "$LAUNCH_SCRIPT" << EOF

#!/bin/bash
# Launch script for ${PACKAGE_NAME}

colcon build
source install/setup.bash
ros2 launch ${PACKAGE_NAME} demo.launch.py
EOF

chmod +x "$LAUNCH_SCRIPT"
echo "Created launch script: $LAUNCH_SCRIPT"
