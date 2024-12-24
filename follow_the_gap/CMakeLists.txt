# CMakeLists.txt for the follow_the_gap package

cmake_minimum_required(VERSION 3.5)
project(follow_the_gap)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add the executable
add_executable(follow_the_gap_node src/follow_the_gap_node.cpp)

# Link dependencies
ament_target_dependencies(follow_the_gap_node
  rclcpp
  sensor_msgs
  geometry_msgs
  ackermann_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS follow_the_gap_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if you have any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Ament package macros
ament_package()

