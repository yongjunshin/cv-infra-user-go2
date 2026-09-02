// patrol_manager_node.cpp — the patrol mission state machine.
//
// It serves this app's OWN goal interface, and drives nav2 underneath it:
//
//   a client  --(go2_msgs/action/Patrol @ /patrol)-->  patrol_manager
//   patrol_manager --(nav2_msgs/action/NavigateToPose @ /navigate_to_pose)-->  nav2
//
// A Patrol goal says WHAT to find ("person", or empty for "any configured class"); it
// never says where. Nav2 keeps its own action under its own standard name, so plain
// point-to-point navigation stays available to anyone who wants it — this node is a
// client of it, not a disguise for it.
//
// ── The mission ────────────────────────────────────────────────────────────────────
//   RECEIVED  goal accepted (a target CLASS, not a pose)
//     -> PROBE       is there perception at all? (a dead camera aborts — see below)
//     -> SEARCHING   drive the app's own patrol route, watching /targets
//     -> APPROACHING two legs: line up on the robot-target line, then walk STRAIGHT in
//                    to the standoff (why two: see beginApproach — measured)
//     -> HOLDING     keep it centred and large on screen for `confirm_hold_s`
//     -> SUCCEED     and then STOP asking nav2 for anything (silence = the robot stands)
//
// ── Why SEARCHING drives instead of turning on the spot (MEASURED) ─────────────────
// This robot's locomotion policy executes in-place yaw at ~6 % of the commanded rate and
// sub-0.2 m/s commands at 5-23 %. A search strategy built on pivoting or creeping would
// stall on THIS robot no matter how good the planner is. So the strategy is a driven
// sweep: a short route of waypoints whose headings come out of the travel direction, and
// an approach that ends facing the target because it drove straight at it. Nav2 stays the
// single producer of autonomy velocities: this node never publishes any.
//
// ── Perception is REQUIRED ────────────────────────────────────────────────────────
// A patrol with no camera is not a degraded patrol, it is a misconfiguration. When no
// /detections and/or no camera_info arrive within `perception_probe_s`, the mission
// ABORTS with exactly that reason instead of pretending: a robot that walks a route
// blind has not searched for anything, and reporting anything but failure would be a
// claim this node did not earn. To drive somewhere without perception, send nav2 a
// NavigateToPose goal directly — which is precisely why its action keeps its own name.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <go2_msgs/action/patrol.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "go2_patrol_manager/patrol_logic.hpp"

namespace go2_patrol_manager
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;   // nav2 CLIENT side
using Patrol = go2_msgs::action::Patrol;                    // the app's own goal interface
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Patrol>;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

enum class State { IDLE, PROBE, SEARCHING, APPROACHING, HOLDING };
enum class NavState { NONE, REQUESTED, PENDING, ACTIVE, SUCCEEDED, FAILED };
//: Approach legs: line up on the robot-target line first, then walk straight in.
enum class ApproachStage { LINEUP, FINAL };
enum class SearchPhase { STRAIGHT, TURNING };  // bounce mode's two legs

const char * stateName(State state)
{
  switch (state) {
    case State::IDLE:
      return "IDLE";
    case State::PROBE:
      return "PROBE";
    case State::SEARCHING:
      return "SEARCHING";
    case State::APPROACHING:
      return "APPROACHING";
    case State::HOLDING:
      return "HOLDING";
  }
  return "?";
}

class PatrolManager : public rclcpp::Node
{
public:
  PatrolManager()
  : Node("go2_patrol_manager")
  {
    // --- parameters: every knob declared, with the measurement behind its default ---
    const auto patrol_action = declare_parameter<std::string>("patrol_action", "/patrol");
    const auto nav2_action = declare_parameter<std::string>("nav2_action", "/navigate_to_pose");
    const auto targets_topic = declare_parameter<std::string>("targets_topic", "/targets");
    const auto detections_topic = declare_parameter<std::string>("detections_topic", "/detections");
    const auto camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    // What a Patrol goal may ask for. This list is what an empty `target_class` means,
    // and what a non-empty one is validated against.
    // ⚠ go2_target_tracker declares the SAME parameter with the SAME default: it is the
    // node that decides which classes ever reach /targets. Override only ONE of the two
    // and a goal class passes the check here but can never be published — the mission
    // then walks the whole route and ends in a route-exhausted abort, which looks like a
    // perception problem and is not one. Change both, or neither.
    target_classes_ = declare_parameter<std::vector<std::string>>(
      "target_classes", std::vector<std::string>{"person", "chair"});
    min_confidence_ = declare_parameter<double>("min_confidence", 0.5);

    // The patrol ROUTE is the app's own configuration, like its map — a Patrol goal names
    // a class, never a place. Default = the warehouse aisle this app ships a map for: a
    // short driven sweep with a 0.5 m lateral offset so the camera pans across the aisle
    // while walking.
    const auto flat_waypoints = declare_parameter<std::vector<double>>(
      "search_waypoints", std::vector<double>{-5.5, 0.8, -6.0, 1.2});
    if (!parseWaypoints(flat_waypoints, waypoints_)) {
      throw std::runtime_error(
        "search_waypoints must be a non-empty flat list of [x, y] pairs (even length)");
    }

    // ★ 2.0 m standoff — the single most important number in this node, and the one that
    // GEOMETRY GOT WRONG. The first draft reasoned from optics ("stand 1.2 m away so the
    // 0.877 m chair is whole and large in the 640x480 frame") and the live run failed:
    // the hold condition could never be met. Measuring the actual detector against the
    // actual asset while walking in (one frame per sim second):
    //
    //   distance   5.5   4.6   4.1   3.2   2.7   2.5   2.2   1.7   1.3   <=1.2
    //   chair conf 0.19  0.76  0.36  0.84  0.83  0.87  0.79  0.84  0.46  0.10-0.38
    //                                                             ^^^^ collapses; the
    //   model reads the near, frame-filling chair as `bench` instead — which is why
    //   yolo11n's "chair" is not a distance-free property.
    //
    // So the working band for THIS target is ~1.7-3.2 m and the standoff sits in the
    // middle of it.
    standoff_m_ = declare_parameter<double>("standoff_m", 2.0);
    // The line-up ring: the approach first drives HERE, then straight in to the standoff
    // (see beginApproach for the measurement that made two legs necessary). 3.2 m is the
    // far end of the measured detection band and leaves a 1.2 m straight final run — long
    // enough for the planner to settle onto the target bearing, short enough to stay
    // inside the aisle's 2.7 m clearance.
    lineup_m_ = declare_parameter<double>("lineup_m", 3.2);
    lineup_skip_margin_m_ = declare_parameter<double>("lineup_skip_margin_m", 0.3);
    // A line-up leg shorter than this is un-walkable for this locomotion policy (short
    // goals draw creep-speed commands that fall into its <0.2 m/s dead zone — see the
    // beginApproach measurement); such approaches go FINAL directly.
    min_lineup_leg_m_ = declare_parameter<double>("min_lineup_leg_m", 1.5);
    // The final leg drives at a point INSIDE the standoff ring and is cancelled when the
    // ring is crossed — see tickApproaching. 0.6 m from the target is short of the
    // standoff ring, so the robot is unambiguously still walking toward the target when
    // we stop it. It is never actually reached (and must not be: 1.4 m is inside the band
    // where this detector stops calling the chair a chair).
    approach_goal_m_ = declare_parameter<double>("approach_goal_m", 1.4);
    confirm_hold_s_ = declare_parameter<double>("confirm_hold_s", 5.0);
    // 0.35 * 640 = +-224 px = +-31 deg of the 82.25 deg horizontal FOV.
    hold_center_tol_frac_ = declare_parameter<double>("hold_center_tol_frac", 0.35);
    // 0.25 of 480 px = 120 px. MEASURED box heights for the chair at the working
    // distances: 213 px @ 1.7 m, 169 px @ 2.15 m, 147 px @ 2.5 m, 130 px @ 2.7 m — so at
    // the 2.0 m standoff there is ~50 % of margin, and the threshold still rejects a
    // target seen from across the warehouse (64 px @ 5.5 m).
    hold_min_height_ratio_ = declare_parameter<double>("hold_min_height_ratio", 0.25);
    hold_timeout_s_ = declare_parameter<double>("hold_timeout_s", 12.0);
    // A confirmation hold must survive a dropped frame. MEASURED: at 1.5 m the chair's
    // YOLO confidence oscillates across the 0.5 threshold from frame to frame (the same
    // framing sensitivity the table above measures), so a hold timer that resets on the
    // first miss can never reach 5 s. The claim we make is "on screen for essentially
    // all of the last `confirm_hold_s`", and this is the "essentially".
    hold_grace_s_ = declare_parameter<double>("hold_grace_s", 1.0);
    max_approach_retries_ = declare_parameter<int>("max_approach_retries", 2);
    // Re-aim the approach when the target ESTIMATE moves this far (measured: a static
    // chair's estimate travels ~1.5 m while AMCL converges — see tickApproaching), with a
    // minimum period so a jittering estimate cannot preempt nav2 every tick.
    retarget_threshold_m_ = declare_parameter<double>("retarget_threshold_m", 0.3);
    retarget_min_period_s_ = declare_parameter<double>("retarget_min_period_s", 1.0);
    // How long to wait for ANY perception before aborting: a camera-less patrol is a
    // configuration error, not a degraded mission.
    perception_probe_s_ = declare_parameter<double>("perception_probe_s", 5.0);
    // A whole approach (line-up + final, plus its bounded retries) that has not reached the
    // hold by now was chasing a target that is no longer there — go back to searching. A
    // real approach is ~10-20 s of sim time (measured); this leaves generous margin.
    approach_timeout_s_ = declare_parameter<double>("approach_timeout_s", 45.0);
    // After an approach gives up, the target is usually STILL on /targets, so searching
    // would re-confirm it and re-approach the same unreachable spot forever while the robot
    // stands still (MEASURED livelock: SEARCH->APPROACH->timeout->SEARCH every 45 s, the
    // whole time inside nav2's dead-zone creep). So ignore a just-failed target for a
    // cooldown: the bounce search then actually walks away and explores, and may re-find it
    // from a reachable angle later.
    retarget_cooldown_s_ = declare_parameter<double>("retarget_cooldown_s", 20.0);
    retarget_cooldown_r_ = declare_parameter<double>("retarget_cooldown_r", 1.5);
    // A detection older than this is not evidence of what is on screen NOW (detector runs
    // at 5 Hz sim-time, so this is 5 missed frames).
    detection_stale_s_ = declare_parameter<double>("detection_stale_s", 1.0);
    // 0 = disabled: the action client owns the mission budget and can cancel this goal
    // whenever it likes. Set a value as a safety net for unattended runs.
    mission_timeout_s_ = declare_parameter<double>("mission_timeout_s", 0.0);
    tick_period_s_ = declare_parameter<double>("tick_period_s", 0.2);

    // --- search strategy -------------------------------------------------------------
    // "bounce" (default): walk straight until the scan says something is close ahead,
    // then arc-turn onto a random clear heading (not the one we came from) and walk
    // straight again — a coverage walk that needs no route. "route": the fixed
    // `search_waypoints` patrol. Both hand over to the same APPROACHING the moment a
    // class-matching target is confirmed.
    search_mode_ = declare_parameter<std::string>("search_mode", "bounce");
    if (search_mode_ != "bounce" && search_mode_ != "route") {
      throw std::runtime_error("search_mode must be 'bounce' or 'route', got: " + search_mode_);
    }
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    // Published only while SEARCHING in bounce mode. twist_mux arbitrates it at priority
    // 50 — above idle nav2 chatter (10), below a human (100). ⚠ In this phase the
    // manager, not nav2, is the driver, so nav2's collision_monitor does NOT gate these
    // commands: the pre-collision guarantee is `bounce_trigger_m` itself.
    const auto cmd_vel_search_topic =
      declare_parameter<std::string>("cmd_vel_search_topic", "cmd_vel_search");
    // 0.4 m/s: comfortably above the policy's <0.2 m/s dead zone.
    search_speed_m_s_ = declare_parameter<double>("search_speed_m_s", 0.4);
    // Bounce when the forward window reports less than this. Budget: at 0.4 m/s the arc
    // turn below has radius ~0.3 m (0.25 / 0.8), so 1.2 m leaves the whole maneuver in
    // free space with ~2x margin.
    bounce_trigger_m_ = declare_parameter<double>("bounce_trigger_m", 1.2);
    // A candidate heading must have this much room to be worth walking into — more than
    // the trigger, so a fresh leg does not begin already needing to bounce.
    bounce_clear_m_ = declare_parameter<double>("bounce_clear_m", 1.5);
    // The "ahead" window watched while walking straight: +-15 deg.
    bounce_forward_halfwidth_rad_ =
      declare_parameter<double>("bounce_forward_halfwidth_rad", 0.26);
    // Headings within 45 deg of straight-behind are never chosen: that is the ground the
    // robot just covered, and bouncing back onto it is how a random walk ping-pongs.
    bounce_reverse_exclude_rad_ =
      declare_parameter<double>("bounce_reverse_exclude_rad", 0.79);
    // A candidate heading is a corridor, not a gap: every ray within +-this many beams
    // must be clear (25 beams x 0.1125 deg/beam ~ +-2.8 deg... widened by the clear
    // window in metres this maps to at bounce_clear_m).
    bounce_window_halfwidth_ = declare_parameter<int>("bounce_window_halfwidth", 60);
    // The turn is an ARC, never a pivot: this policy executes in-place yaw at ~6 % of
    // the commanded rate but tracks it at ~91 % while walking. 0.25 m/s stays above the
    // dead zone; 0.8 rad/s is the app's own wz ceiling.
    bounce_turn_speed_m_s_ = declare_parameter<double>("bounce_turn_speed_m_s", 0.25);
    bounce_turn_wz_ = declare_parameter<double>("bounce_turn_wz", 0.8);
    bounce_heading_tol_rad_ = declare_parameter<double>("bounce_heading_tol_rad", 0.17);
    // A random walk has no "route exhausted" — this is its honest end instead (sim
    // seconds; ~3x the measured route-mode mission).
    search_timeout_s_ = declare_parameter<double>("search_timeout_s", 180.0);
    // 0 = seed from hardware entropy. Any other value makes the bounce sequence
    // reproducible for debugging a specific wander.
    const auto bounce_seed = declare_parameter<int>("bounce_seed", 0);
    rng_.seed(
      bounce_seed == 0 ? std::random_device{}() : static_cast<unsigned int>(bounce_seed));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    targets_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      targets_topic, rclcpp::QoS(5).reliable(),
      [this](vision_msgs::msg::Detection3DArray::SharedPtr msg) { targets_ = std::move(msg); });
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_topic, rclcpp::QoS(5).reliable(),
      [this](vision_msgs::msg::Detection2DArray::SharedPtr msg) { onDetections(*msg); });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::QoS(1).best_effort(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        image_width_ = static_cast<double>(msg->width);
        image_height_ = static_cast<double>(msg->height);
      });
    // The bounce search's eyes-forward: only the latest scan matters, so sensor QoS.
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_ = std::move(msg); });
    search_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(cmd_vel_search_topic, rclcpp::QoS(5));

    // Latched (transient-local) so a client that subscribes AFTER the mission started still
    // gets the current state as its first message — the whole point of state recovery.
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "/patrol_state", rclcpp::QoS(1).transient_local());
    // A goal-id-free stop: a reloaded UI has lost the action goal id it would need to
    // cancel, so it needs a way to stop the running mission by name. Any message here
    // stands the robot down.
    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/patrol_cancel", rclcpp::QoS(1),
      [this](std_msgs::msg::Empty::SharedPtr) { onExternalStop(); });

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action);
    action_server_ = rclcpp_action::create_server<Patrol>(
      this, patrol_action,
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Patrol::Goal> goal) {
        return handleGoal(uuid, goal);
      },
      [this](const std::shared_ptr<ServerGoalHandle> handle) { return handleCancel(handle); },
      [this](const std::shared_ptr<ServerGoalHandle> handle) { handleAccepted(handle); });

    // Sim-time timer (not a wall timer): every duration this node reasons about — the
    // hold, the probe, the timeouts — runs on the node's clock, which is the simulator's
    // /clock whenever use_sim_time is set.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(tick_period_s_), [this]() { tick(); });

    std::string route;
    for (const auto & wp : waypoints_) {
      route += (route.empty() ? "" : " -> ") + std::string("(") + std::to_string(wp.x) + ", " +
               std::to_string(wp.y) + ")";
    }
    RCLCPP_INFO(
      get_logger(),
      "go2_patrol_manager up: serving %s, driving %s | standoff=%.2f m hold=%.1f s "
      "(centre<=%.2f W, height>=%.2f H) search=%s%s%s",
      patrol_action.c_str(), nav2_action.c_str(), standoff_m_, confirm_hold_s_,
      hold_center_tol_frac_, hold_min_height_ratio_, search_mode_.c_str(),
      search_mode_ == "route" ? " route: " : "",
      search_mode_ == "route" ? route.c_str() : "");

    publishState();  // latch the initial IDLE so a UI connecting before any mission sees it
  }

private:
  // ---------------------------------------------------------------- action server ---
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Patrol::Goal> goal)
  {
    if (state_ != State::IDLE) {
      RCLCPP_WARN(get_logger(), "rejecting goal: a patrol mission is already running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // A class this node was never configured for cannot appear on /targets, so accepting
    // it would buy a route-length wait for a guaranteed failure. Say so now instead.
    if (!goal->target_class.empty() &&
        std::find(target_classes_.begin(), target_classes_.end(), goal->target_class) ==
          target_classes_.end()) {
      RCLCPP_WARN(
        get_logger(), "rejecting goal: unknown target_class '%s' (configured: %s)",
        goal->target_class.c_str(), classList().c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(
      get_logger(), "goal accepted: target_class='%s' (empty = any configured class)",
      goal->target_class.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<ServerGoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "cancel requested — stopping nav2 and standing down");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /// Operator stop from a client that does not hold the action goal id (a reloaded UI has
  /// lost it). End the running mission and stand down; the latched state topic then tells
  /// every connected UI it is over.
  void onExternalStop()
  {
    if (state_ == State::IDLE) {
      return;
    }
    RCLCPP_INFO(get_logger(), "external stop received — standing down");
    finish(false, "stopped by the operator");
  }

  void handleAccepted(const std::shared_ptr<ServerGoalHandle> handle)
  {
    goal_handle_ = handle;
    goal_class_ = handle->get_goal()->target_class;
    mission_start_s_ = nowSeconds();
    probe_start_s_ = mission_start_s_;
    approach_retries_ = 0;
    waypoint_index_ = 0;
    setState(State::PROBE, "checking perception is alive before searching");
  }

  void finish(bool success, const std::string & reason)
  {
    cancelNav();
    stopSearchDrive();
    auto result = std::make_shared<Patrol::Result>();
    result->found = success;
    result->message = reason;  // every outcome, not just the failures, says why
    if (success) {
      result->target_pose.header.frame_id = map_frame_;
      result->target_pose.header.stamp = now();
      result->target_pose.pose.position.x = target_x_;
      result->target_pose.pose.position.y = target_y_;
      result->target_pose.pose.position.z = target_z_;
      result->target_pose.pose.orientation.w = 1.0;  // a position, not a heading
    }
    if (goal_handle_ != nullptr) {
      if (goal_handle_->is_canceling()) {
        RCLCPP_INFO(get_logger(), "mission CANCELED: %s", reason.c_str());
        goal_handle_->canceled(result);
      } else if (success) {
        RCLCPP_INFO(get_logger(), "mission SUCCEEDED: %s", reason.c_str());
        goal_handle_->succeed(result);
      } else {
        RCLCPP_ERROR(get_logger(), "mission ABORTED: %s", reason.c_str());
        goal_handle_->abort(result);
      }
    }
    goal_handle_.reset();
    setState(State::IDLE, reason);
  }

  // ---------------------------------------------------------------- nav2 client ----
  void requestNav(const Pose2 & pose, const char * what)
  {
    desired_nav_ = pose;
    nav_state_ = NavState::REQUESTED;
    RCLCPP_INFO(
      get_logger(), "nav2 goal (%s): (%.2f, %.2f) yaw %.2f", what, pose.x, pose.y, pose.yaw);
  }

  void sendNav()
  {
    ++nav_seq_;
    const auto seq = nav_seq_;
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = map_frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = desired_nav_.x;
    goal.pose.pose.position.y = desired_nav_.y;
    goal.pose.pose.orientation.z = std::sin(desired_nav_.yaw * 0.5);
    goal.pose.pose.orientation.w = std::cos(desired_nav_.yaw * 0.5);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this, seq](ClientGoalHandle::SharedPtr handle) {
      if (seq != nav_seq_) {
        return;  // a superseded goal answering late must not move the state machine
      }
      if (handle == nullptr) {
        RCLCPP_WARN(get_logger(), "nav2 rejected the goal");
        nav_state_ = NavState::FAILED;
      } else {
        nav_goal_handle_ = handle;
        nav_state_ = NavState::ACTIVE;
      }
    };
    options.result_callback = [this, seq](const ClientGoalHandle::WrappedResult & result) {
      if (seq != nav_seq_) {
        return;
      }
      nav_state_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED) ? NavState::SUCCEEDED
                                                                        : NavState::FAILED;
    };
    nav_client_->async_send_goal(goal, options);
    nav_state_ = NavState::PENDING;
  }

  /// Drop the inner nav2 goal, whatever state it is in.
  ///
  /// ⚠ CANCELLING A FINISHED GOAL THROWS. `async_cancel_goal` raises
  /// `UnknownGoalHandleError` once the client has forgotten the handle (i.e. after the
  /// result arrived), and an exception thrown inside a timer callback calls
  /// std::terminate. MEASURED: nav2 reported "Goal succeeded", this node called finish()
  /// -> cancelNav() with the completed handle, and the process died with exit -6 — after
  /// which the Patrol goal never returned a result and the mission could only end in a
  /// timeout. Two guards, because the state check alone can race with a result arriving
  /// between the check and the call.
  void cancelNav()
  {
    ++nav_seq_;  // invalidate any in-flight callbacks for the goal we are dropping
    const bool live = nav_state_ == NavState::PENDING || nav_state_ == NavState::ACTIVE;
    if (nav_goal_handle_ != nullptr && live) {
      try {
        nav_client_->async_cancel_goal(nav_goal_handle_);
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError & exc) {
        RCLCPP_DEBUG(
          get_logger(), "nav2 goal was already finished when we tried to cancel it: %s",
          exc.what());
      }
    }
    nav_goal_handle_.reset();
    nav_state_ = NavState::NONE;
  }

  // ---------------------------------------------------------------- perception ----
  void onDetections(const vision_msgs::msg::Detection2DArray & msg)
  {
    // Liveness is CLASS-BLIND on purpose: any detection frame proves the camera ->
    // detector pipeline is running. Asking "did we see a person?" here would confuse an
    // empty warehouse with a dead camera, and only one of those is a configuration error.
    perception_seen_ = true;
    last_detection_s_ = nowSeconds();
    screen_ok_ = false;
    for (const auto & det : msg.detections) {
      if (det.results.empty()) {
        continue;
      }
      const auto & hypothesis = det.results.front().hypothesis;
      if (!classMatches(hypothesis.class_id, goal_class_, target_classes_) ||
          hypothesis.score < min_confidence_) {
        continue;
      }
      if (screenConditionOk(
            det.bbox.center.position.x, det.bbox.size_y, image_width_, image_height_,
            hold_center_tol_frac_, hold_min_height_ratio_)) {
        screen_ok_ = true;
        screen_detail_ = hypothesis.class_id;
        screen_center_px_ = det.bbox.center.position.x;
        screen_height_px_ = det.bbox.size_y;
        break;
      }
    }
  }

  bool perceptionAlive() const
  {
    return perception_seen_ && image_width_ > 0.0 && image_height_ > 0.0;
  }

  bool screenConditionHeld() const
  {
    return screen_ok_ && (nowSeconds() - last_detection_s_) <= detection_stale_s_;
  }

  // ---------------------------------------------------------------- helpers -------
  double nowSeconds() const { return now().seconds(); }

  /// The configured target classes, comma-separated — for log lines only.
  std::string classList() const
  {
    std::string out;
    for (const auto & c : target_classes_) {
      out += (out.empty() ? "" : ",") + c;
    }
    return out;
  }

  bool robotPose(Pose2 & pose)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      pose.x = tf.transform.translation.x;
      pose.y = tf.transform.translation.y;
      pose.yaw = tf2::getYaw(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no %s -> %s transform yet: %s", map_frame_.c_str(),
        base_frame_.c_str(), exc.what());
      return false;
    }
  }

  /// Nearest CONFIRMED target of the mission's class(es), in the map frame.
  ///
  /// "no targets yet" and "targets, but none of the class we were asked for" are the same
  /// answer here — false — and the search simply continues.
  bool nearestTarget(const Pose2 & robot, double & tx, double & ty, double & tz) const
  {
    if (targets_ == nullptr) {
      return false;
    }
    bool found = false;
    double best = std::numeric_limits<double>::max();
    for (const auto & det : targets_->detections) {
      if (det.results.empty()) {
        continue;
      }
      const auto & hyp = det.results.front();
      if (!classMatches(hyp.hypothesis.class_id, goal_class_, target_classes_)) {
        continue;
      }
      const auto & p = hyp.pose.pose.position;
      if (onCooldown(p.x, p.y)) {
        continue;  // a target we just failed on — keep exploring, do not re-lock on it
      }
      const double d = distance(robot.x, robot.y, p.x, p.y);
      if (d < best) {
        best = d;
        tx = p.x;
        ty = p.y;
        tz = p.z;
        found = true;
      }
    }
    return found;
  }

  void setState(State next, const std::string & why)
  {
    if (next != state_) {
      RCLCPP_INFO(get_logger(), "%s -> %s (%s)", stateName(state_), stateName(next), why.c_str());
    }
    state_ = next;
    publishState();
  }

  /// The mission state as a latched topic, so a UI that connects (or reconnects, or is
  /// reloaded) mid-mission learns immediately that a patrol is running and for what —
  /// the client that sent the goal is not the only thing that should know. Format:
  /// "STATE" or "STATE class" (e.g. "SEARCHING person"); "IDLE" means no mission.
  void publishState()
  {
    if (state_pub_ == nullptr) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = stateName(state_);
    if (state_ != State::IDLE && !goal_class_.empty()) {
      msg.data += " " + goal_class_;
    }
    state_pub_->publish(msg);
  }

  void publishFeedback(const Pose2 & robot)
  {
    if (goal_handle_ == nullptr) {
      return;
    }
    auto feedback = std::make_shared<Patrol::Feedback>();
    feedback->state = stateName(state_);
    feedback->robot_pose.header.frame_id = map_frame_;
    feedback->robot_pose.header.stamp = now();
    feedback->robot_pose.pose.position.x = robot.x;
    feedback->robot_pose.pose.position.y = robot.y;
    feedback->robot_pose.pose.orientation.z = std::sin(robot.yaw * 0.5);
    feedback->robot_pose.pose.orientation.w = std::cos(robot.yaw * 0.5);
    feedback->elapsed_s = static_cast<float>(nowSeconds() - mission_start_s_);
    goal_handle_->publish_feedback(feedback);
  }

  // ---------------------------------------------------------------- state machine --
  void tick()
  {
    if (state_ == State::IDLE || goal_handle_ == nullptr) {
      return;
    }
    if (goal_handle_->is_canceling()) {
      finish(false, "cancel requested by the client");
      return;
    }
    if (mission_timeout_s_ > 0.0 && (nowSeconds() - mission_start_s_) > mission_timeout_s_) {
      finish(false, "mission timeout (patrol_manager.mission_timeout_s)");
      return;
    }
    if (nav_state_ == NavState::REQUESTED) {
      if (!nav_client_->action_server_is_ready()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "waiting for the inner nav2 action server");
        return;
      }
      sendNav();
    }

    Pose2 robot;
    const bool have_pose = robotPose(robot);
    if (have_pose) {
      publishFeedback(robot);
    }

    switch (state_) {
      case State::PROBE:
        tickProbe(robot, have_pose);
        break;
      case State::SEARCHING:
        tickSearching(robot, have_pose);
        break;
      case State::APPROACHING:
        tickApproaching(robot, have_pose);
        break;
      case State::HOLDING:
        tickHolding();
        break;
      case State::IDLE:
        break;
    }
  }

  void tickProbe(const Pose2 & robot, bool have_pose)
  {
    if (perceptionAlive()) {
      setState(State::SEARCHING, "perception is alive — patrolling");
      search_start_s_ = nowSeconds();
      search_phase_ = SearchPhase::STRAIGHT;
      bounce_count_ = 0;
      if (search_mode_ == "route" && have_pose) {
        sendSearchGoal(robot);
      }
      // bounce mode sends nothing here: it drives itself from the tick.
      return;
    }
    if ((nowSeconds() - probe_start_s_) < perception_probe_s_) {
      return;
    }
    finish(
      false, "no perception after " + std::to_string(perception_probe_s_) +
               " s (no /detections and/or no camera_info) — a patrol needs a live camera; "
               "aborting");
  }

  void sendSearchGoal(const Pose2 & robot)
  {
    const auto & wp = waypoints_[waypoint_index_];
    Pose2 pose{wp.x, wp.y, travelYaw(robot.x, robot.y, wp, robot.yaw)};
    requestNav(pose, "search waypoint");
  }

  void tickSearching(const Pose2 & robot, bool have_pose)
  {
    if (!have_pose) {
      return;
    }
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    if (nearestTarget(robot, tx, ty, tz)) {
      cancelNav();
      stopSearchDrive();  // bounce mode: hand the wheel back before nav2 takes it
      target_x_ = tx;
      target_y_ = ty;
      target_z_ = tz;
      approach_retries_ = 0;
      setState(
        State::APPROACHING, "confirmed target at (" + std::to_string(tx) + ", " +
                              std::to_string(ty) + ")");
      beginApproach(robot, "approach");
      return;
    }
    // The one not-found end of the mission, shared by both search modes: the whole-mission
    // budget from the first time we started looking. Everything else (a lost target, an
    // unreachable one, an exhausted route) sends the robot back out rather than home.
    if ((nowSeconds() - search_start_s_) > search_timeout_s_) {
      finish(
        false, "search budget exhausted (" + std::to_string(search_timeout_s_) +
                 " s) without confirming a target");
      return;
    }
    if (search_mode_ == "bounce") {
      tickSearchingBounce(robot);
      return;
    }
    if (nav_state_ == NavState::NONE) {
      // We entered SEARCHING before TF could tell us where we were standing, so the first
      // waypoint was never sent. Send it now that the pose is available.
      sendSearchGoal(robot);
      return;
    }
    if (nav_state_ == NavState::SUCCEEDED || nav_state_ == NavState::FAILED) {
      const bool failed = nav_state_ == NavState::FAILED;
      ++waypoint_index_;
      if (waypoint_index_ >= waypoints_.size()) {
        // Route done, still nothing — loop it. The budget check above is what ends the
        // mission, so a patrol keeps walking its round until it finds the target or runs
        // out of time, never after a single lap.
        waypoint_index_ = 0;
        RCLCPP_INFO(get_logger(), "route completed without a target — starting another lap");
      } else if (failed) {
        RCLCPP_WARN(get_logger(), "nav2 could not reach that waypoint — moving to the next one");
      }
      sendSearchGoal(robot);
    }
  }

  /// The bounce search: straight until the scan says something is close ahead, then an
  /// ARC turn onto a random clear non-backtracking heading, then straight again. The
  /// choice itself is the pure `pickBounceHeading` (tested on CPU); this method only
  /// owns timing, the RNG draw, and the Twist publishing.
  void tickSearchingBounce(const Pose2 & robot)
  {
    // (The whole-mission search budget is checked in tickSearching, common to both modes.)
    if (scan_ == nullptr) {
      // No scan = no pre-collision guarantee = do not move. The mux releases /cmd_vel
      // 0.5 s after our last message, so silence here IS the stop.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "bounce search is waiting for %s — not moving",
        scan_sub_->get_topic_name());
      return;
    }
    const auto & scan = *scan_;
    // Forward clearance: the nearest finite return within +-the forward window.
    double fwd = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      if (std::fabs(a) > bounce_forward_halfwidth_rad_) {
        continue;
      }
      const float r = scan.ranges[i];
      if (std::isfinite(r) && static_cast<double>(r) < fwd) {
        fwd = static_cast<double>(r);
      }
    }
    geometry_msgs::msg::Twist cmd;
    if (search_phase_ == SearchPhase::TURNING) {
      const double err = shortestAngle(robot.yaw, bounce_target_yaw_);
      if (std::fabs(err) <= bounce_heading_tol_rad_) {
        search_phase_ = SearchPhase::STRAIGHT;
        RCLCPP_INFO(
          get_logger(), "bounce %d complete — straight along yaw %.2f", bounce_count_,
          bounce_target_yaw_);
      } else {
        cmd.linear.x = bounce_turn_speed_m_s_;
        cmd.angular.z = std::copysign(bounce_turn_wz_, err);
        search_pub_->publish(cmd);
        return;
      }
    }
    if (fwd < bounce_trigger_m_) {
      const double u01 = std::uniform_real_distribution<double>(0.0, 1.0)(rng_);
      const auto choice = pickBounceHeading(
        scan.ranges, scan.angle_min, scan.angle_increment, bounce_clear_m_,
        bounce_reverse_exclude_rad_, bounce_window_halfwidth_, u01);
      if (!choice.found) {
        finish(
          false, "boxed in after " + std::to_string(bounce_count_) +
                   " bounce(s): no non-backtracking heading has " +
                   std::to_string(bounce_clear_m_) + " m of clearance");
        return;
      }
      ++bounce_count_;
      bounce_target_yaw_ = robot.yaw + choice.heading_rad;
      search_phase_ = SearchPhase::TURNING;
      RCLCPP_INFO(
        get_logger(),
        "bounce %d: %.2f m ahead < %.2f m trigger — arc-turning %.0f deg to yaw %.2f",
        bounce_count_, fwd, bounce_trigger_m_, choice.heading_rad * 180.0 / M_PI,
        bounce_target_yaw_);
      cmd.linear.x = bounce_turn_speed_m_s_;
      cmd.angular.z = std::copysign(bounce_turn_wz_, choice.heading_rad);
      search_pub_->publish(cmd);
      return;
    }
    cmd.linear.x = search_speed_m_s_;
    search_pub_->publish(cmd);
  }

  /// One zero Twist, so the robot is not left coasting on the last search command while
  /// the mux's 0.5 s timeout runs down. Harmless when the search never drove.
  void stopSearchDrive()
  {
    if (search_pub_ != nullptr) {
      search_pub_->publish(geometry_msgs::msg::Twist{});
    }
    search_phase_ = SearchPhase::STRAIGHT;
  }

  /// A patrol does not give up because one target could not be reached or held — it goes
  /// back to looking. The ONLY not-found end of the mission is the whole-mission search
  /// budget (checked in tickSearching); a single lost, unreachable, or un-holdable target
  /// just sends the robot back out. `search_start_s_` is deliberately NOT reset here, so
  /// the budget spans the whole mission across any number of find/lose cycles.
  void resumeSearch(const std::string & why)
  {
    cancelNav();
    stopSearchDrive();
    // Remember the target we just could not reach/hold, so the search does not instantly
    // re-lock on it (see retarget_cooldown_s_). This is what turns "resume search" into an
    // actual walk away rather than a standing-still livelock.
    failed_target_x_ = target_x_;
    failed_target_y_ = target_y_;
    failed_target_s_ = nowSeconds();
    waypoint_index_ = 0;  // restart the route (route mode); a no-op walk state in bounce mode
    approach_retries_ = 0;
    RCLCPP_WARN(get_logger(), "%s — resuming the search", why.c_str());
    setState(State::SEARCHING, why + " — resuming search");
  }

  /// A confirmed target we just failed on, still inside its cooldown window? Then the
  /// search must ignore it and keep exploring instead of re-locking on the spot.
  bool onCooldown(double x, double y) const
  {
    return (nowSeconds() - failed_target_s_) < retarget_cooldown_s_ &&
           distance(x, y, failed_target_x_, failed_target_y_) <= retarget_cooldown_r_;
  }

  /// Start (or restart) the approach: line up first, then walk the last stretch straight.
  ///
  /// ⚠ THE MEASURED REASON THIS HAS TWO LEGS. nav2's goal checker here is
  /// position-only (`yaw_goal_tolerance` is effectively disabled — this robot cannot
  /// pivot on the spot) and it stops as soon as the robot is within 0.5 m of
  /// the goal. So the ARRIVAL HEADING is simply the direction the robot happened to be
  /// travelling. The first live chair run ended 0.43 m short of its standoff pose with a
  /// **29 deg heading error** (measured with tf2_echo: robot yaw 117 deg vs target
  /// bearing 88 deg), which put the chair at 484-526 px instead of 320 and made the
  /// on-screen hold flicker in and out until the mission gave up.
  /// The fix does not fight the planner: it makes the LAST leg a straight run along the
  /// target bearing (line-up point -> standoff, both on the robot-target line), so
  /// "where the robot is pointing when it stops" IS the target direction. No in-place
  /// rotation, no creep — the two things this locomotion policy will not execute.
  void beginApproach(const Pose2 & robot, const char * why)
  {
    approach_start_s_ = nowSeconds();  // bounds this approach; a stuck one goes back to search
    const double range = distance(robot.x, robot.y, target_x_, target_y_);
    // The line-up point sits on the target->robot ray at lineup_m, so the line-up LEG
    // the robot must walk is only (range - lineup_m). A robot hovering just outside the
    // ring gets a sub-metre leg — and a short goal is exactly where the controller
    // commands creep speeds this policy will not execute (measured live: a 1.1 m
    // line-up leg drew cmd_vel 0.15 m/s -> the dead zone executed 5-23 % of it ->
    // "Failed to make progress" three times and the retry budget burned). A leg
    // shorter than min_lineup_leg_m_ is un-walkable here, so go FINAL directly:
    // the walk-at-the-target run carries full speed and stops on the standoff ring.
    if (range > lineup_m_ + lineup_skip_margin_m_ &&
      range - lineup_m_ >= min_lineup_leg_m_)
    {
      approach_stage_ = ApproachStage::LINEUP;
      requestNav(
        standoffPose(target_x_, target_y_, robot.x, robot.y, lineup_m_, robot.yaw), why);
    } else {
      // Already inside the line-up ring: driving BACKWARDS to line up would be a worse
      // trade than approaching from here.
      approach_stage_ = ApproachStage::FINAL;
      requestNav(
        standoffPose(target_x_, target_y_, robot.x, robot.y, approach_goal_m_, robot.yaw), why);
    }
  }

  void tickApproaching(const Pose2 & robot, bool have_pose)
  {
    // An approach that never converges must not hang the mission: nav2 can stay ACTIVE
    // indefinitely replanning toward a ghost target (one that was confirmed, then lost) and
    // then neither the ring, nor SUCCEEDED, nor FAILED ever fires. Bound it — a stale
    // approach goes back to searching (MEASURED: a lost-target approach hung in APPROACHING
    // and rejected every new goal until the node was restarted).
    if ((nowSeconds() - approach_start_s_) > approach_timeout_s_) {
      resumeSearch("approach did not converge in time");
      return;
    }
    // The target ESTIMATE improves while we walk toward it — it is a projection through
    // AMCL's map->odom, and the filter converges as the robot moves (measured live: a
    // static chair's estimate travelled 1.5 m during the first approach). Re-aim on the
    // LINE-UP leg only: preempting nav2 during the final straight run is what curves it,
    // and the estimate is already converged by then.
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    const double now_s = nowSeconds();
    if (approach_stage_ == ApproachStage::LINEUP && have_pose &&
        nav_state_ != NavState::REQUESTED && nearestTarget(robot, tx, ty, tz) &&
        distance(tx, ty, target_x_, target_y_) > retarget_threshold_m_ &&
        (now_s - last_retarget_s_) >= retarget_min_period_s_) {
      last_retarget_s_ = now_s;
      RCLCPP_INFO(
        get_logger(), "target estimate moved %.2f m -> re-aiming the line-up",
        distance(tx, ty, target_x_, target_y_));
      target_x_ = tx;
      target_y_ = ty;
      target_z_ = tz;
      requestNav(
        standoffPose(target_x_, target_y_, robot.x, robot.y, lineup_m_, robot.yaw),
        "line-up (re-aimed)");
      return;
    }
    // ⚠ STOP ON THE RING, DO NOT ARRIVE AT A POSE. MEASURED (live chair run): nav2's
    // goal checker here is position-only and fires as soon as the robot is within 0.5 m
    // of the goal, so "arriving" leaves the robot with whatever heading it had — 22-29 deg
    // off the target in both live runs, which puts the chair at ~500 px instead of 320 and
    // costs enough confidence to break the hold. Walking THROUGH the standoff and
    // cancelling on the ring stops the robot while it is still travelling toward the
    // target, and a robot that cannot strafe is by construction pointing where it walks.
    if (approach_stage_ == ApproachStage::FINAL && have_pose &&
        distance(robot.x, robot.y, target_x_, target_y_) <= standoff_m_) {
      RCLCPP_INFO(
        get_logger(), "standoff ring reached (%.2f m from the target) — stopping here",
        distance(robot.x, robot.y, target_x_, target_y_));
      cancelNav();
      hold_enter_s_ = nowSeconds();
      hold_since_s_ = -1.0;
      last_screen_ok_s_ = -1e9;
      setState(State::HOLDING, "standoff ring reached");
      return;
    }
    if (nav_state_ == NavState::SUCCEEDED) {
      if (approach_stage_ == ApproachStage::LINEUP) {
        if (!have_pose) {
          return;
        }
        // Refresh the estimate one last time, then walk straight at it.
        if (nearestTarget(robot, tx, ty, tz)) {
          target_x_ = tx;
          target_y_ = ty;
          target_z_ = tz;
        }
        approach_stage_ = ApproachStage::FINAL;
        requestNav(
          standoffPose(target_x_, target_y_, robot.x, robot.y, approach_goal_m_, robot.yaw),
          "final approach (walk at the target, stop at the standoff ring)");
        return;
      }
      hold_enter_s_ = nowSeconds();
      hold_since_s_ = -1.0;
      last_screen_ok_s_ = -1e9;
      setState(State::HOLDING, "reached the approach goal without crossing the ring");
      return;
    }
    if (nav_state_ == NavState::FAILED) {
      if (approach_retries_ >= max_approach_retries_) {
        resumeSearch("could not reach the target after retries");
        return;
      }
      ++approach_retries_;
      if (!have_pose) {
        return;
      }
      RCLCPP_WARN(get_logger(), "approach failed — retry %d", approach_retries_);
      beginApproach(robot, "approach retry");
    }
  }

  void tickHolding()
  {
    const double now_s = nowSeconds();
    if (screenConditionHeld()) {
      last_screen_ok_s_ = now_s;
      if (hold_since_s_ < 0.0) {
        hold_since_s_ = now_s;
        RCLCPP_INFO(
          get_logger(), "target on screen (%s, centre %.0f px, height %.0f px) — holding %.1f s",
          screen_detail_.c_str(), screen_center_px_, screen_height_px_, confirm_hold_s_);
      }
      if ((now_s - hold_since_s_) >= confirm_hold_s_) {
        finish(
          true, "target confirmed on screen for " + std::to_string(confirm_hold_s_) +
                  " s at the standoff pose");
      }
      return;
    }
    if (hold_since_s_ >= 0.0 && (now_s - last_screen_ok_s_) > hold_grace_s_) {
      RCLCPP_WARN(
        get_logger(), "target off screen for more than %.1f s — hold timer reset", hold_grace_s_);
      hold_since_s_ = -1.0;
    }
    if ((now_s - hold_enter_s_) > hold_timeout_s_) {
      if (approach_retries_ >= max_approach_retries_) {
        resumeSearch("could not hold the target on screen");
        return;
      }
      ++approach_retries_;
      Pose2 robot;
      if (!robotPose(robot)) {
        return;
      }
      RCLCPP_WARN(
        get_logger(), "hold condition unmet for %.1f s — re-approaching (retry %d)",
        hold_timeout_s_, approach_retries_);
      beginApproach(robot, "approach retry after failed hold");
      setState(State::APPROACHING, "re-approaching to fix the viewing geometry");
    }
  }

  // ---------------------------------------------------------------- members -------
  std::string map_frame_;
  std::string base_frame_;
  std::vector<std::string> target_classes_;
  std::string goal_class_;  // this mission's class; empty = any of target_classes_
  double min_confidence_{0.5};
  std::vector<Waypoint> waypoints_;
  double standoff_m_{2.0};
  double lineup_m_{3.2};
  double lineup_skip_margin_m_{0.3};
  double min_lineup_leg_m_{1.5};
  double approach_goal_m_{1.4};
  double confirm_hold_s_{5.0};
  double hold_center_tol_frac_{0.35};
  double hold_min_height_ratio_{0.25};
  double hold_timeout_s_{12.0};
  double hold_grace_s_{1.0};
  double last_screen_ok_s_{-1e9};
  int max_approach_retries_{2};
  double retarget_threshold_m_{0.3};
  double retarget_min_period_s_{1.0};
  double last_retarget_s_{-1e9};
  double perception_probe_s_{5.0};
  double approach_timeout_s_{45.0};
  double approach_start_s_{0.0};
  double retarget_cooldown_s_{20.0};
  double retarget_cooldown_r_{1.5};
  double failed_target_x_{0.0};
  double failed_target_y_{0.0};
  double failed_target_s_{-1e9};
  double detection_stale_s_{1.0};
  double mission_timeout_s_{0.0};
  double tick_period_s_{0.2};

  std::string search_mode_{"bounce"};
  double search_speed_m_s_{0.4};
  double bounce_trigger_m_{1.2};
  double bounce_clear_m_{1.5};
  double bounce_forward_halfwidth_rad_{0.26};
  double bounce_reverse_exclude_rad_{0.79};
  int bounce_window_halfwidth_{60};
  double bounce_turn_speed_m_s_{0.25};
  double bounce_turn_wz_{0.8};
  double bounce_heading_tol_rad_{0.17};
  double search_timeout_s_{180.0};
  double search_start_s_{0.0};
  double bounce_target_yaw_{0.0};
  int bounce_count_{0};
  SearchPhase search_phase_{SearchPhase::STRAIGHT};
  std::mt19937 rng_;

  State state_{State::IDLE};
  ApproachStage approach_stage_{ApproachStage::LINEUP};
  NavState nav_state_{NavState::NONE};
  Pose2 desired_nav_;
  uint64_t nav_seq_{0};
  std::size_t waypoint_index_{0};
  int approach_retries_{0};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double mission_start_s_{0.0};
  double probe_start_s_{0.0};
  double hold_enter_s_{0.0};
  double hold_since_s_{-1.0};

  bool perception_seen_{false};
  double last_detection_s_{0.0};
  bool screen_ok_{false};
  std::string screen_detail_;
  double screen_center_px_{0.0};
  double screen_height_px_{0.0};
  double image_width_{0.0};
  double image_height_{0.0};

  vision_msgs::msg::Detection3DArray::SharedPtr targets_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr targets_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr search_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  ClientGoalHandle::SharedPtr nav_goal_handle_;
  rclcpp_action::Server<Patrol>::SharedPtr action_server_;
  std::shared_ptr<ServerGoalHandle> goal_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace go2_patrol_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_patrol_manager::PatrolManager>());
  rclcpp::shutdown();
  return 0;
}
