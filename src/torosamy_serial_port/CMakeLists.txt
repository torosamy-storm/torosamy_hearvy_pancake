cmake_minimum_required(VERSION 3.10)
project(torosamy_serial_port)
set(CMAKE_CXX_STANDARD 17)
# 添加编译选项
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
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
target_link_libraries(${PROJECT_NAME}
        yaml-cpp
        /opt/torosamy_robomaster_sdk/lib/libtorosamy_data_sdk.so
        /opt/torosamy_robomaster_sdk/lib/libtorosamy_uart_sdk.so
        /opt/torosamy_robomaster_sdk/lib/libtorosamy_module_sdk.so
)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        /opt/torosamy_robomaster_sdk/include
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
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