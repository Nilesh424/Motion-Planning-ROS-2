cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_controller)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Include directories (if you have any headers)
include_directories(
  include
)

##########################
# Add executables for each CPP file
##########################

# 1. Pure Pursuit default node
add_executable(pure_pursuit_node src/pure_pursuit_controller_node.cpp)
ament_target_dependencies(pure_pursuit_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ackermann_msgs
)

# 2. Pure Pursuit with delay node
add_executable(pure_pursuit_with_delay_node src/pure_pursuit_controller_node_with_delay.cpp)
ament_target_dependencies(pure_pursuit_with_delay_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ackermann_msgs
)

# 3. Pure Pursuit lingering lookahead node
add_executable(pure_pursuit_lingering_node src/pure_pursuit_controller_node_with_lingering_lookahead_point.cpp)
ament_target_dependencies(pure_pursuit_lingering_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ackermann_msgs
)

##########################
# Install the executables
##########################
install(TARGETS
  pure_pursuit_node
  pure_pursuit_with_delay_node
  pure_pursuit_lingering_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

