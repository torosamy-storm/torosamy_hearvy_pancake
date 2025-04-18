cmake_minimum_required(VERSION 3.5)

project(torosamy_ros_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(torosamy_ros_msgs
  "msg/ObstacleMsg.msg"
  "msg/ObstacleArrayMsg.msg"
  "msg/ReceiveDataMsg.msg"
  "msg/SendDataMsg.msg"
  "msg/RefereeSystemMsg.msg"
  "msg/NavigationClient.msg"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
)

ament_auto_package(
  INSTALL_TO_SHARE
  msg
)