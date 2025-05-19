cmake_minimum_required(VERSION 3.8)
project(bridge)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(bridge_msgs REQUIRED)

# Explicitly specify the Python package path
ament_python_install_package(bridge PACKAGE_DIR scripts)

# Install Python executables
install(PROGRAMS
  scripts/client/rclpy_client.py
  scripts/client/roslib_client.py
  scripts/server/bridge_server.py
  DESTINATION lib/bridge
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()