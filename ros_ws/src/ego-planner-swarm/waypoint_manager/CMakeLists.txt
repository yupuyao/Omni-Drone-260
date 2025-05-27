cmake_minimum_required(VERSION 3.8)
project(waypoint_manager)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(k230_msgs REQUIRED)


# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(include)

# Create executable for waypoint manager
add_executable(waypoint_manager_node src/waypoint_manager.cpp)
target_include_directories(waypoint_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  waypoint_manager_node
  rclcpp
  geometry_msgs
  k230_msgs
)

# Install targets
install(TARGETS
  waypoint_manager_node
  DESTINATION lib/${PROJECT_NAME})

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
