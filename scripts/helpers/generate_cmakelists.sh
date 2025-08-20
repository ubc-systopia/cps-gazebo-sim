#!/bin/bash
# Usage: generate_cmakelists.sh <package_name> <pkg_dir>

PACKAGE_NAME="$1"
PKG_DIR="$2"

OUT_FILE="${PKG_DIR}/CMakeLists.txt"

cat > "$OUT_FILE" << EOF
cmake_minimum_required(VERSION 3.22)
project(${PACKAGE_NAME})

find_package(ament_cmake REQUIRED)

# Install common resource folders if present
if(EXISTS "\${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/\${PROJECT_NAME}
  )
endif()

if(EXISTS "\${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config
    DESTINATION share/\${PROJECT_NAME}
  )
endif()

if(EXISTS "\${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(
    DIRECTORY urdf
    DESTINATION share/\${PROJECT_NAME}
  )
endif()

ament_package()
EOF
