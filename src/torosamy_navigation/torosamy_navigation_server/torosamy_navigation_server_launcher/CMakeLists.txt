cmake_minimum_required(VERSION 3.8)
project(torosamy_navigation_server_launcher)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()


find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)


ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  maps
  PCD
  urdf
)

