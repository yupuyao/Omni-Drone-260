cmake_minimum_required(VERSION 3.8)
project(trajectory_input)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
include_directories("~/ego-planner-swarm/install/quadrotor_msgs/include")

add_executable(trajectory_input_node src/trajectory_input_node.cpp)
add_executable(timer_node src/timer_node.cpp)

ament_target_dependencies(timer_node
  rclcpp
  quadrotor_msgs
)

ament_target_dependencies(trajectory_input_node
  rclcpp
  quadrotor_msgs
)

install(TARGETS
  trajectory_input_node
  timer_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
