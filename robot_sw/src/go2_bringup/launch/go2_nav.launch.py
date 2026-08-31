# go2_nav.launch.py — headless Nav2 bringup for the Unitree Go2 patrol SUT (stage-0).
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
# STAGE-0 / NO MAP YET. The static map for the go2 warehouse is a platform C1
# deliverable and has not landed. `map` therefore defaults to '' and, while it is
# empty, LOCALIZATION IS DISABLED: map_server and amcl do not start (upstream
# `use_localization` argument, present in nav2_bringup 1.3.12 — verified in the
# pinned deb, not assumed). The nav stack itself still comes up, which is exactly
# what the U0 headless dry-run checks. Passing `map:=/path/to/map.yaml` re-enables
# localization with no change to this file (that is U1's first move).
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

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    # Localization (map_server + amcl) runs only when a map was actually supplied.
    # Evaluates to the string 'True'/'False', which is what bringup_launch.py's own
    # PythonExpression condition concatenates.
    use_localization = PythonExpression(['"', map_yaml, '" != ""'])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use the external simulator /clock (REQ-EXEC-003).",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description=(
                    "Full path to the static map yaml. EMPTY (stage-0 default) = run "
                    "the nav stack WITHOUT localization; map_server/amcl do not start."
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
                    "[go2_bringup] no `map:=` given -> localization DISABLED "
                    "(map_server/amcl not started). stage-0 default; pass a map yaml "
                    "once the C1 map artifact lands."
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
