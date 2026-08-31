// patrol_manager_node.cpp — the patrol mission state machine.
//
// It EXPOSES the platform-facing goal interface itself:
//
//   cv-infra  --(nav2_msgs/action/NavigateToPose @ /navigate_to_pose)-->  patrol_manager
//   patrol_manager --(the same action type @ /nav2/navigate_to_pose)-->  nav2 bt_navigator
//
// (decision 2026-08-31 D4). The adapter contract does not move an inch: cv-infra keeps
// sending exactly the goal it sent to bare nav2 in U1, and nav2 itself is remapped INWARD
// by go2_patrol.launch.py. That is the whole point — a second, very different SUT mission
// on an unchanged platform contract.
//
// ── The mission ────────────────────────────────────────────────────────────────────
//   RECEIVED  goal accepted (its POSE is not used to search — see below)
//     -> PROBE       is there perception at all? (decides patrol vs the nav-only fallback)
//     -> SEARCHING   drive the app's own patrol route, watching /targets
//     -> APPROACHING two legs: line up on the robot-target line, then walk STRAIGHT in
//                    to the standoff (why two: see beginApproach — measured)
//     -> HOLDING     keep it centred and large on screen for `confirm_hold_s`
//     -> SUCCEED     and then STOP asking nav2 for anything (silence = the robot stands)
//
// ── Why the goal pose is not used for searching (master plan §1-5) ─────────────────
// The scenario's goal is the STANDOFF POINT in front of the target (2.0 m — a measured
// distance, see below) — it is the verdict anchor for the platform's oracles, not a hint
// for the robot. This app finds the
// target with the camera and computes its own standoff; the only thing the incoming goal
// does in patrol mode is start the mission. The single exception is the NAV-ONLY fallback
// below, which is loud about it.
//
// ── Why SEARCHING drives instead of turning on the spot (AR-16 / AR-18, MEASURED) ──
// This robot's locomotion policy executes in-place yaw at ~6 % of the commanded rate and
// sub-0.2 m/s commands at 5-23 % (platform C2b §6-1, U1 §6-2 sweep). A search strategy
// built on pivoting or creeping would stall on THIS SUT no matter how good the planner
// is. So the strategy is a driven sweep: a short route of waypoints whose headings come
// out of the travel direction, and an approach that ends facing the target because it
// drove straight at it. Nav2 stays the single producer of /cmd_vel (patrol app DESIGN):
// this node never publishes velocities.
//
// ── The NAV-ONLY fallback, and why it is not a hole in the verdict ────────────────
// The same image serves the nav scenarios (T0/TA — no camera declared in their
// `adapter_config`, so the runner publishes no camera streams and the detector has
// nothing to read) and the patrol scenarios (TB — camera declared). With no perception
// at all, "patrol" is not a thing this app can do, so it degrades to driving to the goal
// it was given and says so LOUDLY in the log. In TB the camera is declared and perception
// is alive within a second of the goal, so the fallback is never taken there; if it ever
// were, the log line is the evidence and the state machine never reports a HOLD it did
// not do.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "go2_patrol_manager/patrol_logic.hpp"

namespace go2_patrol_manager
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

enum class State { IDLE, PROBE, SEARCHING, APPROACHING, HOLDING, NAV_ONLY };
enum class NavState { NONE, REQUESTED, PENDING, ACTIVE, SUCCEEDED, FAILED };
//: Approach legs: line up on the robot-target line first, then walk straight in.
enum class ApproachStage { LINEUP, FINAL };

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
    case State::NAV_ONLY:
      return "NAV_ONLY";
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
    const auto patrol_action = declare_parameter<std::string>("patrol_action", "/navigate_to_pose");
    const auto nav2_action =
      declare_parameter<std::string>("nav2_action", "/nav2/navigate_to_pose");
    const auto targets_topic = declare_parameter<std::string>("targets_topic", "/targets");
    const auto detections_topic = declare_parameter<std::string>("detections_topic", "/detections");
    const auto camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    target_classes_ = declare_parameter<std::vector<std::string>>(
      "target_classes", std::vector<std::string>{"person", "chair"});
    min_confidence_ = declare_parameter<double>("min_confidence", 0.5);

    // The patrol ROUTE is the app's own configuration (like its map), NOT the mission
    // goal. Default = the warehouse aisle this repo's scenarios use: a short driven sweep
    // with a 0.5 m lateral offset so the camera pans across the aisle while walking.
    // Both waypoints stay >= 2.0 m away from the TB goal on purpose: a run that only
    // walks the route (perception dead) must FAIL `reached_goal`, never coast to a pass.
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
    // actual asset while walking in (U3 report §6-2, one frame per sim second):
    //
    //   distance   5.5   4.6   4.1   3.2   2.7   2.5   2.2   1.7   1.3   <=1.2
    //   chair conf 0.19  0.76  0.36  0.84  0.83  0.87  0.79  0.84  0.46  0.10-0.38
    //                                                             ^^^^ collapses; the
    //   model reads the near, frame-filling chair as `bench` instead (exactly the AR-24
    //   failure mode, and the reason yolo11n's "chair" is not a distance-free property).
    //
    // So the working band for THIS target is ~1.7-3.2 m and the standoff sits in the
    // middle of it. The scenarios put their goal at the same 2.0 m in front of the target,
    // i.e. the verdict anchor is where a robot that actually found the target stands.
    standoff_m_ = declare_parameter<double>("standoff_m", 2.0);
    // The line-up ring: the approach first drives HERE, then straight in to the standoff
    // (see beginApproach for the measurement that made two legs necessary). 3.2 m is the
    // far end of the measured detection band and leaves a 1.2 m straight final run — long
    // enough for the planner to settle onto the target bearing, short enough to stay
    // inside the aisle's 2.7 m clearance.
    lineup_m_ = declare_parameter<double>("lineup_m", 3.2);
    lineup_skip_margin_m_ = declare_parameter<double>("lineup_skip_margin_m", 0.3);
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
    // YOLO confidence oscillates across the 0.5 threshold from frame to frame (AR-24 —
    // the same framing sensitivity C0/U2 measured), so a hold timer that resets on the
    // first miss can never reach 5 s. The claim we make is "on screen for essentially
    // all of the last `confirm_hold_s`", and this is the "essentially".
    hold_grace_s_ = declare_parameter<double>("hold_grace_s", 1.0);
    max_approach_retries_ = declare_parameter<int>("max_approach_retries", 2);
    // Re-aim the approach when the target ESTIMATE moves this far (measured: a static
    // chair's estimate travels ~1.5 m while AMCL converges — see tickApproaching), with a
    // minimum period so a jittering estimate cannot preempt nav2 every tick.
    retarget_threshold_m_ = declare_parameter<double>("retarget_threshold_m", 0.3);
    retarget_min_period_s_ = declare_parameter<double>("retarget_min_period_s", 1.0);
    // How long to wait for ANY perception before deciding this is a nav-only scenario.
    perception_probe_s_ = declare_parameter<double>("perception_probe_s", 5.0);
    // A detection older than this is not evidence of what is on screen NOW (detector runs
    // at 5 Hz sim-time, so this is 5 missed frames).
    detection_stale_s_ = declare_parameter<double>("detection_stale_s", 1.0);
    // 0 = disabled: on the verification path the platform owns the mission budget
    // (scenario.timeout_s, sim-time) and cancels this goal itself. This is a standalone
    // safety net for hand runs.
    mission_timeout_s_ = declare_parameter<double>("mission_timeout_s", 0.0);
    tick_period_s_ = declare_parameter<double>("tick_period_s", 0.2);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    targets_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      targets_topic, rclcpp::QoS(5).reliable(),
      [this](geometry_msgs::msg::PoseArray::SharedPtr msg) { targets_ = std::move(msg); });
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_topic, rclcpp::QoS(5).reliable(),
      [this](vision_msgs::msg::Detection2DArray::SharedPtr msg) { onDetections(*msg); });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::QoS(1).best_effort(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        image_width_ = static_cast<double>(msg->width);
        image_height_ = static_cast<double>(msg->height);
      });

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action);
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, patrol_action,
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal) {
        return handleGoal(uuid, goal);
      },
      [this](const std::shared_ptr<ServerGoalHandle> handle) { return handleCancel(handle); },
      [this](const std::shared_ptr<ServerGoalHandle> handle) { handleAccepted(handle); });

    // Sim-time timer (not a wall timer): every duration this node reasons about — the
    // hold, the probe, the timeouts — is sim time, the same clock the platform's budget
    // and oracles use (D-F).
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
      "(centre<=%.2f W, height>=%.2f H) route: %s",
      patrol_action.c_str(), nav2_action.c_str(), standoff_m_, confirm_hold_s_,
      hold_center_tol_frac_, hold_min_height_ratio_, route.c_str());
  }

private:
  // ---------------------------------------------------------------- action server ---
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    if (state_ != State::IDLE) {
      RCLCPP_WARN(get_logger(), "rejecting goal: a patrol mission is already running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(
      get_logger(), "goal accepted: (%.2f, %.2f) frame=%s — searching with the camera, not with it",
      goal->pose.pose.position.x, goal->pose.pose.position.y,
      goal->pose.header.frame_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<ServerGoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "cancel requested — stopping nav2 and standing down");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<ServerGoalHandle> handle)
  {
    goal_handle_ = handle;
    requested_goal_ = handle->get_goal()->pose;
    mission_start_s_ = nowSeconds();
    probe_start_s_ = mission_start_s_;
    approach_retries_ = 0;
    waypoint_index_ = 0;
    setState(State::PROBE, "waiting for perception before choosing patrol vs nav-only");
  }

  void finish(bool success, const std::string & reason)
  {
    cancelNav();
    auto result = std::make_shared<NavigateToPose::Result>();
    if (goal_handle_ != nullptr) {
      if (goal_handle_->is_canceling()) {
        RCLCPP_INFO(get_logger(), "mission CANCELED: %s", reason.c_str());
        goal_handle_->canceled(result);
      } else if (success) {
        RCLCPP_INFO(get_logger(), "mission SUCCEEDED: %s", reason.c_str());
        goal_handle_->succeed(result);
      } else {
        result->error_msg = reason;
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
  /// std::terminate. MEASURED, on the nav-only path: nav2 reported "Goal succeeded", this
  /// node called finish() -> cancelNav() with the completed handle, and the process died
  /// with exit -6 — after which the platform-facing action never returned a result and the
  /// mission could only end in a timeout. Two guards, because the state check alone can
  /// race with a result arriving between the check and the call.
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
    perception_seen_ = true;
    last_detection_s_ = nowSeconds();
    screen_ok_ = false;
    for (const auto & det : msg.detections) {
      if (det.results.empty()) {
        continue;
      }
      const auto & hypothesis = det.results.front().hypothesis;
      const bool is_target =
        std::find(target_classes_.begin(), target_classes_.end(), hypothesis.class_id) !=
        target_classes_.end();
      if (!is_target || hypothesis.score < min_confidence_) {
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

  /// Nearest CONFIRMED target to the robot, in the map frame.
  bool nearestTarget(const Pose2 & robot, double & tx, double & ty) const
  {
    if (targets_ == nullptr || targets_->poses.empty()) {
      return false;
    }
    double best = std::numeric_limits<double>::max();
    for (const auto & pose : targets_->poses) {
      const double d = distance(robot.x, robot.y, pose.position.x, pose.position.y);
      if (d < best) {
        best = d;
        tx = pose.position.x;
        ty = pose.position.y;
      }
    }
    return true;
  }

  void setState(State next, const std::string & why)
  {
    if (next != state_) {
      RCLCPP_INFO(get_logger(), "%s -> %s (%s)", stateName(state_), stateName(next), why.c_str());
    }
    state_ = next;
  }

  void publishFeedback(const Pose2 & robot)
  {
    if (goal_handle_ == nullptr) {
      return;
    }
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose.header.frame_id = map_frame_;
    feedback->current_pose.header.stamp = now();
    feedback->current_pose.pose.position.x = robot.x;
    feedback->current_pose.pose.position.y = robot.y;
    feedback->current_pose.pose.orientation.z = std::sin(robot.yaw * 0.5);
    feedback->current_pose.pose.orientation.w = std::cos(robot.yaw * 0.5);
    feedback->navigation_time = rclcpp::Duration::from_seconds(nowSeconds() - mission_start_s_);
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
      case State::NAV_ONLY:
        tickNavOnly();
        break;
      case State::IDLE:
        break;
    }
  }

  void tickProbe(const Pose2 & robot, bool have_pose)
  {
    if (perceptionAlive()) {
      setState(State::SEARCHING, "perception is alive — patrolling");
      if (have_pose) {
        sendSearchGoal(robot);
      }
      return;
    }
    if ((nowSeconds() - probe_start_s_) < perception_probe_s_) {
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "NO PERCEPTION after %.1f s (no /detections and/or no camera_info) — this scenario "
      "declares no camera, so the patrol degrades to plain navigation to the requested goal. "
      "A patrol scenario must NOT take this path.",
      perception_probe_s_);
    Pose2 goal_pose{
      requested_goal_.pose.position.x, requested_goal_.pose.position.y,
      tf2::getYaw(requested_goal_.pose.orientation)};
    requestNav(goal_pose, "nav-only fallback");
    setState(State::NAV_ONLY, "no perception available");
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
    if (nearestTarget(robot, tx, ty)) {
      cancelNav();
      target_x_ = tx;
      target_y_ = ty;
      approach_retries_ = 0;
      setState(
        State::APPROACHING, "confirmed target at (" + std::to_string(tx) + ", " +
                              std::to_string(ty) + ")");
      beginApproach(robot, "approach");
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
        finish(
          false,
          "patrol route exhausted without confirming a target (last waypoint " +
            std::string(failed ? "failed" : "reached") + ")");
        return;
      }
      if (failed) {
        RCLCPP_WARN(get_logger(), "nav2 could not reach that waypoint — moving to the next one");
      }
      sendSearchGoal(robot);
    }
  }

  /// Start (or restart) the approach: line up first, then walk the last stretch straight.
  ///
  /// ⚠ THE MEASURED REASON THIS HAS TWO LEGS. nav2's goal checker here is
  /// position-only (`yaw_goal_tolerance` is effectively disabled — this robot cannot
  /// pivot on the spot, AR-16/18) and it stops as soon as the robot is within 0.5 m of
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
    const double range = distance(robot.x, robot.y, target_x_, target_y_);
    if (range > lineup_m_ + lineup_skip_margin_m_) {
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
    // The target ESTIMATE improves while we walk toward it — it is a projection through
    // AMCL's map->odom, and the filter converges as the robot moves (measured live: a
    // static chair's estimate travelled 1.5 m during the first approach). Re-aim on the
    // LINE-UP leg only: preempting nav2 during the final straight run is what curves it,
    // and the estimate is already converged by then.
    double tx = 0.0;
    double ty = 0.0;
    const double now_s = nowSeconds();
    if (approach_stage_ == ApproachStage::LINEUP && have_pose &&
        nav_state_ != NavState::REQUESTED && nearestTarget(robot, tx, ty) &&
        distance(tx, ty, target_x_, target_y_) > retarget_threshold_m_ &&
        (now_s - last_retarget_s_) >= retarget_min_period_s_) {
      last_retarget_s_ = now_s;
      RCLCPP_INFO(
        get_logger(), "target estimate moved %.2f m -> re-aiming the line-up",
        distance(tx, ty, target_x_, target_y_));
      target_x_ = tx;
      target_y_ = ty;
      requestNav(
        standoffPose(target_x_, target_y_, robot.x, robot.y, lineup_m_, robot.yaw),
        "line-up (re-aimed)");
      return;
    }
    // ⚠ STOP ON THE RING, DO NOT ARRIVE AT A POSE. MEASURED (live chair run): nav2's
    // goal checker here is position-only and fires as soon as the robot is within 0.5 m
    // of the goal, so "arriving" leaves the robot with whatever heading it had — 22-29 deg
    // off the target in both live runs, which puts the chair at ~500 px instead of 320 and
    // costs enough confidence to break the hold (AR-24). Walking THROUGH the standoff and
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
        if (nearestTarget(robot, tx, ty)) {
          target_x_ = tx;
          target_y_ = ty;
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
        finish(false, "nav2 could not reach the approach pose after retries");
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
        finish(false, "target never met the on-screen hold condition at the standoff pose");
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

  void tickNavOnly()
  {
    if (nav_state_ == NavState::SUCCEEDED) {
      finish(true, "nav-only fallback reached the requested goal");
    } else if (nav_state_ == NavState::FAILED) {
      finish(false, "nav-only fallback could not reach the requested goal");
    }
  }

  // ---------------------------------------------------------------- members -------
  std::string map_frame_;
  std::string base_frame_;
  std::vector<std::string> target_classes_;
  double min_confidence_{0.5};
  std::vector<Waypoint> waypoints_;
  double standoff_m_{2.0};
  double lineup_m_{3.2};
  double lineup_skip_margin_m_{0.3};
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
  double detection_stale_s_{1.0};
  double mission_timeout_s_{0.0};
  double tick_period_s_{0.2};

  State state_{State::IDLE};
  ApproachStage approach_stage_{ApproachStage::LINEUP};
  NavState nav_state_{NavState::NONE};
  Pose2 desired_nav_;
  uint64_t nav_seq_{0};
  std::size_t waypoint_index_{0};
  int approach_retries_{0};
  double target_x_{0.0};
  double target_y_{0.0};
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

  geometry_msgs::msg::PoseStamped requested_goal_;
  geometry_msgs::msg::PoseArray::SharedPtr targets_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr targets_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  ClientGoalHandle::SharedPtr nav_goal_handle_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
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
