cmake_minimum_required(VERSION 3.5)
project(improved_artificial_potential_field_algorithm)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories (if you have any headers in include/)
include_directories(
  include
)

# Add the executable
add_executable(improved_apf_node src/improved_artificial_potential_field_algorithm_node.cpp)

# Link dependencies
ament_target_dependencies(improved_apf_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS improved_apf_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

