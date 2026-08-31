cmake_minimum_required(VERSION 3.8)
project(go2_target_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(target_tracker_node src/target_tracker_node.cpp)
target_include_directories(target_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(target_tracker_node
  rclcpp geometry_msgs sensor_msgs vision_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS target_tracker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  # The pure projection/association math only — no ROS, no sim, no model. Runs on any
  # CPU in milliseconds, which is the point: these are the functions that fail silently.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_projection test/test_projection.cpp)
  target_include_directories(test_projection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
