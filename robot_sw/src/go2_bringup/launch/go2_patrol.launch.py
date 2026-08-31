# go2_patrol.launch.py — the FULL patrol SUT: nav stack + perception + mission.
#
#   cv-infra ──(NavigateToPose @ /navigate_to_pose)──> go2_patrol_manager
#                                                         │
#                       go2_detector ──/detections──> go2_target_tracker ──/targets──┘
#                                                         │
#                          (NavigateToPose @ /nav2/navigate_to_pose) ──> nav2 bt_navigator
#
# The one structural trick here is the REMAP: `go2_nav.launch.py` (and through it the
# upstream `nav2_bringup`) is included inside a scoped group carrying
# `SetRemap('/navigate_to_pose' -> '/nav2/navigate_to_pose')`, so nav2's own action server
# moves INWARD and the patrol manager takes over the platform-facing name (decision D4).
# cv-infra's `adapter_config.goal_interface` therefore does not change by a single
# character between the U1 nav scenarios and the U3 patrol scenarios — the same contract,
# a completely different mission behind it.
#
# ⚠ The remap has to survive nav2's COMPOSED bring-up (`use_composition:=True` is the
# upstream default in Jazzy). It does: launch_ros applies `ros_remaps` to composable nodes
# as well as plain ones (`load_composable_nodes.py`), and this launch's live check is
# `ros2 action list` showing `/nav2/navigate_to_pose` next to our own
# `/navigate_to_pose` — verified, not assumed (U3 report).
#
# Local development (master plan §1-8): nothing here mentions cv-infra. Bring it up
# against the platform's dev-world, or any sim publishing the same topics, and drive it
# with a stock `ros2 action send_goal /navigate_to_pose ...`.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

#: nav2's own goal action, moved out of the way so the patrol manager can own the
#: platform-facing name. Both halves of the pair live here, once.
PLATFORM_GOAL_ACTION = "/navigate_to_pose"
INNER_NAV2_ACTION = "/nav2/navigate_to_pose"


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
                description="Use the external simulator /clock (REQ-EXEC-003).",
            ),
            DeclareLaunchArgument(
                "use_detector",
                default_value="True",
                description=(
                    "Start the YOLO detector. `use_detector:=False` is the perception-dead "
                    "rehearsal: the manager then takes its NAV-ONLY fallback, which is the "
                    "same path the camera-less T0/TA scenarios take."
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
            # Accepted for SUT-contract compatibility (M8 §3.9 documents `rviz:=false`);
            # this launch is headless by construction and never starts RViz.
            DeclareLaunchArgument(
                "rviz",
                default_value="False",
                description="No-op: headless launch never starts RViz.",
            ),
            # ── nav stack, with its goal action pushed inward ──────────────────────
            GroupAction(
                [
                    SetRemap(src=PLATFORM_GOAL_ACTION, dst=INNER_NAV2_ACTION),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "map": LaunchConfiguration("map"),
                        }.items(),
                    ),
                ]
            ),
            # ── perception: pixels only, no mission knowledge ──────────────────────
            # The detector keeps its own wide default threshold (0.25): it does not know
            # what a target is. The 0.5 that AR-9 fixed for TARGETS is applied by the two
            # nodes that do know — the tracker (`min_confidence`) and the manager's
            # on-screen hold condition.
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
                parameters=[
                    sim_time_param,
                    {
                        "patrol_action": PLATFORM_GOAL_ACTION,
                        "nav2_action": INNER_NAV2_ACTION,
                    },
                ],
            ),
        ]
    )
