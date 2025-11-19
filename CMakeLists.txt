cmake_minimum_required(VERSION 3.8)
project(eos_robotics)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Check compiler support for C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Rust integration - we'll use a custom approach since Rust isn't natively supported in ROS2
# We'll create C++ nodes that interface with the Rust library

# Add the Eos C++ ROS2 node
add_executable(eos_ros_node 
  src/eos_ros_node.cpp
  src/neural_bridge.cpp
  src/navigation_controller.cpp
)

# Link dependencies
ament_target_dependencies(eos_ros_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Install the executable
install(TARGETS
  eos_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install world files for Gazebo
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts for neural network bridge
install(PROGRAMS
  scripts/neural_bridge.py
  scripts/sensor_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Export include directories
ament_export_include_directories(include)

# Finalize package
ament_package()