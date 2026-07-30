cmake_minimum_required(VERSION 3.8)
project(ros2_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/LidarPose.msg")
ament_export_dependencies(rosidl_default_runtime)

add_executable(lidar_data_node src/lidar_data_node.cpp)
add_executable(lidar_to_px4_bridge src/lidar_to_px4_bridge.cpp)

target_include_directories(lidar_data_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lidar_data_node ${cpp_typesupport_target})

ament_target_dependencies(lidar_data_node rclcpp geometry_msgs tf2 tf2_geometry_msgs nav_msgs)
ament_target_dependencies(lidar_to_px4_bridge rclcpp geometry_msgs nav_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS lidar_data_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS lidar_to_px4_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})

ament_package()
