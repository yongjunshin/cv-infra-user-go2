cmake_minimum_required(VERSION 3.8)
project(go2_bringup)

# Install-only package: no targets, no compilation. It ships launch + params that
# compose upstream nav2_bringup (do-not-reinvent).
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
