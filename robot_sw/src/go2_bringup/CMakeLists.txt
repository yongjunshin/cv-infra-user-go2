cmake_minimum_required(VERSION 3.8)
project(go2_bringup)

# Install-only package: no targets, no compilation. It ships launch + params +
# the vendored map and behavior tree that compose upstream nav2_bringup
# (do-not-reinvent).
#
# ⚠ params/nav2_params.yaml resolves `$(find-pkg-share go2_bringup)/behavior_trees/...`
# at launch time, so `behavior_trees` MUST stay installed — dropping it from this
# list turns into a bt_navigator startup failure, not a build error.
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
