# go2_patrol.launch.py — the FULL app: nav stack + perception + mission.
#
#   a client ──(go2_msgs/action/Patrol @ /patrol)──> go2_patrol_manager
#                                                         │
#                       go2_detector ──/detections──> go2_target_tracker ──/targets──┘
#                                                         │
#                        (NavigateToPose @ /navigate_to_pose) ──> nav2 bt_navigator
#
# Every server here keeps its own name: the app's mission interface is /patrol, and nav2's
# NavigateToPose stays at /navigate_to_pose where anyone expects to find it. Nothing is
# remapped, so `ros2 action list` reads like the diagram above.
#
# Drive it with:
#   ros2 action send_goal /patrol go2_msgs/action/Patrol "{target_class: person}"
#
# Local development is 1st-class: bring this up against any sim publishing the sensor
# topics, or a real robot, and send the same goal.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_launch = PathJoinSubstitution(
        [FindPackageShare("go2_bringup"), "launch", "go2_nav.launch.py"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_detector = LaunchConfiguration("use_detector")
    sim_time_param = {"use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use the external simulator /clock.",
            ),
            DeclareLaunchArgument(
                "use_detector",
                default_value="True",
                description=(
                    "Start the YOLO detector. `use_detector:=False` is the perception-dead "
                    "rehearsal: the manager's PROBE times out and the mission ABORTS loudly "
                    "instead of driving blind."
                ),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("go2_bringup"),
                        "maps",
                        "carter_warehouse_navigation.yaml",
                    ]
                ),
                description="Static map yaml, forwarded to go2_nav.launch.py.",
            ),
            # ── nav stack (+ twist_mux), under its own names ───────────────────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": LaunchConfiguration("map"),
                }.items(),
            ),
            # ── perception: pixels only, no mission knowledge ──────────────────────
            # The detector keeps its own wide default threshold (0.25): it does not know
            # what a target is. The 0.5 that TARGETS need is applied by the two nodes that
            # do know — the tracker (`min_confidence`) and the manager's on-screen hold
            # condition.
            Node(
                package="go2_detector",
                executable="detector_node",
                name="go2_detector",
                output="screen",
                parameters=[sim_time_param],
                condition=IfCondition(use_detector),
            ),
            Node(
                package="go2_target_tracker",
                executable="target_tracker_node",
                name="go2_target_tracker",
                output="screen",
                parameters=[sim_time_param],
            ),
            # ── the mission ───────────────────────────────────────────────────────
            # Every threshold this node runs on has exactly one home: the C++ defaults,
            # next to the measurement that produced them. Override per run with
            # `--ros-args -p standoff_m:=...` when experimenting locally.
            Node(
                package="go2_patrol_manager",
                executable="patrol_manager_node",
                name="go2_patrol_manager",
                output="screen",
                parameters=[sim_time_param],
            ),
        ]
    )
