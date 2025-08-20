#!/bin/bash
# DEPRECATED: use generate_robot_description_wrapper.sh instead.
# This shim forwards to the new helper for backward compatibility.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[WARNING] generate_pkg_wrapper_xacro.sh is deprecated; forwarding to generate_robot_description_wrapper.sh" >&2
exec bash "$SCRIPT_DIR/generate_robot_description_wrapper.sh" "$@"
