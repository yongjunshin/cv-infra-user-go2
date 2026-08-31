cmake_minimum_required(VERSION 3.8)
project(go2_patrol_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(patrol_manager_node src/patrol_manager_node.cpp)
target_include_directories(patrol_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(patrol_manager_node
  rclcpp rclcpp_action geometry_msgs sensor_msgs vision_msgs nav2_msgs tf2 tf2_ros
  tf2_geometry_msgs)

install(TARGETS patrol_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  # Standoff geometry + hold condition + route parsing only — the three decisions that
  # can be wrong for an entire GPU run without crashing anything.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_patrol_logic test/test_patrol_logic.cpp)
  target_include_directories(test_patrol_logic PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
