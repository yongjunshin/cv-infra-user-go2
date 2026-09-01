// patrol_logic.hpp — the patrol manager's PURE decisions (no ROS, no sim, no clock).
//
// Three things in the manager can be silently wrong for a whole live run:
//   1. WHERE to stand when a target is found (standoff on the wrong side = the robot
//      drives THROUGH the target instead of stopping in front of it);
//   2. WHICH WAY to face there (the wrong yaw = the target is out of frame and the hold
//      condition can never be met — this robot cannot fix it by turning in place, see
//      below);
//   3. WHEN the target counts as "held on screen" (centred AND large).
// They are pure functions here so they are asserted on a CPU in milliseconds.
//
// ⚠ Why the standoff YAW matters so much on THIS robot (MEASURED): the locomotion policy
// executes in-place yaw at ~6 % of the commanded rate and any command below ~0.2 m/s at
// ~5-23 %. So the app must not plan anything that needs a pivot or a creep: the arrival
// heading has to come out of the DRIVE itself. That is why the standoff pose is always
// "on the segment from the target toward the robot, facing the target" — driving to it is
// a straight approach whose final heading already points at the target.

#ifndef GO2_PATROL_MANAGER__PATROL_LOGIC_HPP_
#define GO2_PATROL_MANAGER__PATROL_LOGIC_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace go2_patrol_manager
{

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Waypoint
{
  double x{0.0};
  double y{0.0};
};

/// Heading from (fx, fy) toward (tx, ty). Undefined for coincident points -> 0 by atan2.
inline double bearing(double fx, double fy, double tx, double ty)
{
  return std::atan2(ty - fy, tx - fx);
}

inline double distance(double ax, double ay, double bx, double by)
{
  return std::hypot(bx - ax, by - ay);
}

/// The pose to observe a target from: `standoff_m` in front of it, on the robot's side,
/// facing it.
///
/// `fallback_yaw` is used only in the degenerate case where the robot is already on top
/// of the target (no direction to derive) — the robot then keeps its current heading
/// instead of the code inventing one.
inline Pose2 standoffPose(
  double target_x, double target_y, double robot_x, double robot_y, double standoff_m,
  double fallback_yaw)
{
  const double dx = robot_x - target_x;
  const double dy = robot_y - target_y;
  const double range = std::hypot(dx, dy);
  if (range < 1e-6) {
    return Pose2{robot_x, robot_y, fallback_yaw};
  }
  const double ux = dx / range;
  const double uy = dy / range;
  Pose2 pose;
  pose.x = target_x + ux * standoff_m;
  pose.y = target_y + uy * standoff_m;
  pose.yaw = bearing(pose.x, pose.y, target_x, target_y);  // stand there LOOKING at it
  return pose;
}

/// Is the target held on screen? ("whole and large" — the condition under which this
/// robot's detector is actually reliable.)
///
/// * centred:  |u - width/2| <= center_tol_frac * width
/// * large:    bbox height / image height >= min_height_ratio
///
/// The "whole" half is bought by the STANDOFF DISTANCE, which was measured
/// against the real detector (see patrol_manager_node.cpp: the chair is read as `chair`
/// at 0.79-0.87 between 1.7 and 3.2 m and as `bench` below ~1.3 m), not asserted here:
/// a 1.73 m person and a 0.877 m chair fill the frame differently at the same distance,
/// and a hold condition tuned to either one's height would be measuring the target, not
/// the robot's behaviour.
inline bool screenConditionOk(
  double center_x_px, double size_y_px, double image_width_px, double image_height_px,
  double center_tol_frac, double min_height_ratio)
{
  if (image_width_px <= 0.0 || image_height_px <= 0.0 || size_y_px <= 0.0) {
    return false;
  }
  const double centre_error = std::fabs(center_x_px - image_width_px * 0.5);
  if (centre_error > center_tol_frac * image_width_px) {
    return false;
  }
  return (size_y_px / image_height_px) >= min_height_ratio;
}

/// Does a reported class satisfy the mission? An empty `goal_class` means "any of the
/// configured target classes"; a non-empty one narrows the mission to exactly that
/// class (exact, case-sensitive string match — the detector emits COCO names verbatim).
inline bool classMatches(
  const std::string & class_id, const std::string & goal_class,
  const std::vector<std::string> & configured)
{
  if (!goal_class.empty()) {
    return class_id == goal_class;
  }
  return std::find(configured.begin(), configured.end(), class_id) != configured.end();
}

/// Flat [x0, y0, x1, y1, ...] -> waypoints. False (and `out` untouched) when the list is
/// empty or has an odd length — a half-written route must be a loud startup failure, not
/// a robot that silently patrols half of it.
inline bool parseWaypoints(const std::vector<double> & flat, std::vector<Waypoint> & out)
{
  if (flat.empty() || (flat.size() % 2U) != 0U) {
    return false;
  }
  std::vector<Waypoint> parsed;
  parsed.reserve(flat.size() / 2U);
  for (std::size_t i = 0; i + 1U < flat.size(); i += 2U) {
    parsed.push_back(Waypoint{flat[i], flat[i + 1U]});
  }
  out.swap(parsed);
  return true;
}

/// Heading to give a search waypoint: along the direction of travel (from where the robot
/// is now, or from the previous waypoint) — never a heading that has to be reached by
/// pivoting on the spot.
inline double travelYaw(double from_x, double from_y, const Waypoint & to, double fallback_yaw)
{
  if (distance(from_x, from_y, to.x, to.y) < 1e-6) {
    return fallback_yaw;
  }
  return bearing(from_x, from_y, to.x, to.y);
}

}  // namespace go2_patrol_manager

#endif  // GO2_PATROL_MANAGER__PATROL_LOGIC_HPP_
