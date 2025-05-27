cmake_minimum_required(VERSION 3.8)
project(k230_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisualData.msg"
)

ament_package()

