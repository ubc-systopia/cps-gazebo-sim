cmake_minimum_required(VERSION 3.8)
project(mesh_objects_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install the launch-time planning-scene publisher as a runnable node
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/scene_publisher.py")
  install(
    PROGRAMS scripts/scene_publisher.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
