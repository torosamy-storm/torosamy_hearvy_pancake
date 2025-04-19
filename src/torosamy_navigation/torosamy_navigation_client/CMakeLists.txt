cmake_minimum_required(VERSION 3.10)
project(torosamy_navigation_client)

# 添加编译选项
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

# 查找依赖项
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(torosamy_ros_msgs REQUIRED)


file(GLOB_RECURSE SERIAL_PORT_SOURCE_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type (Debug or Release)." FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
add_executable(${PROJECT_NAME} ${SERIAL_PORT_SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    yaml-cpp
)


ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "nav2_msgs"
  "rclcpp_action"
  "std_msgs"
  "geometry_msgs"
  "torosamy_ros_msgs"
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
)