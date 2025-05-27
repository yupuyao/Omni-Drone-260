cmake_minimum_required(VERSION 3.8)
project(imu_transform)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Declare the executable
add_executable(imu_transform_node 
  src/imu_transform_node.cpp
)

# Add dependencies
ament_target_dependencies(imu_transform_node
  rclcpp
  std_msgs
  sensor_msgs
  px4_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Install executables
install(TARGETS
  imu_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY
  include/
  DESTINATION include
)

# Run linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()