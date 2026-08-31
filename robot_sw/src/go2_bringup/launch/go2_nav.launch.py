# go2_nav.launch.py — headless Nav2 bringup for the Unitree Go2 patrol SUT.
#
# do-not-reinvent: this is assembly glue, NOT a reimplementation. It composes
# `nav2_bringup/bringup_launch.py` (apt ros-jazzy-nav2-bringup, pinned in the
# Dockerfile) with this package's Nav2 param set. RViz is never started (M8 §3.9 D-O:
# rviz in headless CI is unnecessary and a failure source).
#
# SUT contract surface exposed by this launch (see ../../../../README.md):
#   - accepts use_sim_time (default True) -> external /clock             (REQ-EXEC-003)
#   - nav2 bt_navigator exposes nav2_msgs/action/NavigateToPose
#       @ /navigate_to_pose                                             (REQ-EXEC-007)
#   - subscribes /scan + /odom, publishes /cmd_vel                      (REQ-EXEC-006)
#
# MAP (U1). `map` now defaults to this package's vendored occupancy grid
# (../maps/carter_warehouse_navigation.yaml — see ../maps/README.md for its upstream
# commit + digests and for WHY the carter map is the right map for the go2 scene), so
# map_server + amcl come up by default and the robot localizes in the same frame the
# scenarios express their goals in.
#
# `use_localization:=False` still means "run the nav stack WITHOUT localization"
# (upstream argument, present in nav2_bringup 1.3.12 — verified in the pinned deb, not
# assumed). That is the U0 dry-run mode and stays available for bringing the stack up
# against a sim that has no map.
#
# Local development (the 1st-class requirement, master plan §1-8): this launch is
# plain ROS 2 — `ros2 launch go2_bringup go2_nav.launch.py` works against the
# platform's dev-world (D-7) or any other sim/robot publishing the same topics,
# with no cv-infra in the loop.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_bringup_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("go2_bringup"), "params", "nav2_params.yaml"]
    )
    default_map_file = PathJoinSubstitution(
        [FindPackageShare("go2_bringup"), "maps", "carter_warehouse_navigation.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    # Localization (map_server + amcl) runs unless the caller says otherwise, and is
    # off automatically when the map was blanked out. The default is a Substitution
    # evaluating to the string 'True'/'False', which is what bringup_launch.py's own
    # PythonExpression condition concatenates.
    #
    # It is a real ARGUMENT (not just this expression) because `map:=''` is NOT
    # expressible on the `ros2 launch` command line — the CLI rejects an empty value
    # ("malformed launch argument 'map:=', expected format '<name>:=<value>'",
    # measured 2026-09-01). Without this argument, the no-map bring-up mode would only
    # be reachable from another launch file, which is not where a developer stands.
    map_given = PythonExpression(['"', map_yaml, '" != ""'])
    use_localization = LaunchConfiguration("use_localization")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use the external simulator /clock (REQ-EXEC-003).",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_file,
                description=(
                    "Full path to the static map yaml. Defaults to this package's "
                    "vendored warehouse grid."
                ),
            ),
            DeclareLaunchArgument(
                "use_localization",
                default_value=map_given,
                description=(
                    "Start map_server + amcl. Defaults to True whenever a map is set "
                    "(it is by default). `use_localization:=False` brings up the nav "
                    "stack alone — the no-map mode, e.g. against a sim with no map."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Nav2 params (this package's stock-derived set).",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="True",
                description="Automatically transition the nav2 lifecycle nodes up.",
            ),
            # Accepted for SUT-contract compatibility (M8 §3.9 documents `rviz:=false`).
            # This launch is headless by construction: RViz is never started regardless.
            DeclareLaunchArgument(
                "rviz",
                default_value="False",
                description="No-op: headless launch never starts RViz.",
            ),
            LogInfo(
                condition=UnlessCondition(use_localization),
                msg=(
                    "[go2_bringup] localization DISABLED (map_server/amcl not "
                    "started). The nav stack still comes up; drop "
                    "`use_localization:=False` to localize on the vendored map."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    "map": map_yaml,
                    "use_localization": use_localization,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": autostart,
                }.items(),
            ),
        ]
    )
