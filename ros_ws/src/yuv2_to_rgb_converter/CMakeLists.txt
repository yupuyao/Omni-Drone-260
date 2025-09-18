cmake_minimum_required(VERSION 3.8)
project(yuv2_to_rgb_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# C++ executable
add_executable(yuv2_to_rgb_converter_cpp src/yuv2_to_rgb_converter.cpp)
ament_target_dependencies(yuv2_to_rgb_converter_cpp 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV)

# Install C++ executables
install(TARGETS
  yuv2_to_rgb_converter_cpp
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
