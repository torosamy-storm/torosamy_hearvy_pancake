cmake_minimum_required(VERSION 3.10)
project(torosamy_automatic_aiming)

# 使用 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -fsanitize=address -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# 查找 ROS 依赖项
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(torosamy_ros_msgs REQUIRED)

find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)



file(GLOB_RECURSE AUTOMATIC_AIMING_SOURCE_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)
add_executable(${PROJECT_NAME} ${AUTOMATIC_AIMING_SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME}  rclcpp)
target_link_libraries(${PROJECT_NAME}
        yaml-cpp
        /opt/torosamy_robomaster_sdk/lib/libtorosamy_camera_sdk.so
        /opt/torosamy_robomaster_sdk/lib/libtorosamy_data_sdk.so
        /opt/torosamy_robomaster_sdk/lib/libtorosamy_module_sdk.so
        /opt/torosamy_robomaster_sdk/lib/libtorosamy_uart_sdk.so
        ${OpenCV_LIBS}
        ${EIGEN3_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        /opt/torosamy_robomaster_sdk/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
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
