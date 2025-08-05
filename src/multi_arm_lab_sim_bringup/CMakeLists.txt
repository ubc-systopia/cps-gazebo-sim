cmake_minimum_required(VERSION 3.5)

project(multi_arm_lab_sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(multi_arm_lab_sim_description REQUIRED)
find_package(multi_arm_lab_sim_gazebo REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install project test files
install(
  DIRECTORY
    test/
  DESTINATION share/${PROJECT_NAME}/test
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # run unit tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(multiple_ur_launch test/test_multiple_ur_launch.py)
  ament_add_pytest_test(xsarm_launch test/test_xsarm_launch.py)
  ament_add_pytest_test(environment_launch test/test_environment_launch.py)
endif()

ament_package()
