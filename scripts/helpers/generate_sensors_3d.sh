#!/bin/bash
# Usage: generate_sensors_3d.sh <package_name> <pkg_dir>

PACKAGE_NAME="$1"
PKG_DIR="$2"

out_file="${PKG_DIR}/config/sensors_3d.yaml"

# Determine workspace root (scripts/helpers -> workspace root)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SOURCE_FILE="${WORKSPACE_ROOT}/src/two_arm_moveit_config/config/sensors_3d.yaml"

if [ -f "$SOURCE_FILE" ]; then
	cp "$SOURCE_FILE" "$out_file"
else
	# Fallback minimal file if reference not found
	cat > "$out_file" << 'EOF'
sensors: []
EOF
fi
