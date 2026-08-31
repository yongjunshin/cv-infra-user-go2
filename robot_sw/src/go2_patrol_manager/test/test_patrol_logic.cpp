// CPU unit tests for the patrol manager's pure decisions (no ROS, no sim).
//
// The coordinates below are the ones the TB scenarios actually use: aisle at x = -6.0,
// target at (-6.0, 5.2), verdict goal at the app's own standoff distance 2.0 m in front of
// it at (-6.0, 3.2), robot walking up the aisle from y < 0. Frame = 640x480 (measured
// camera stream). 2.0 m is MEASURED, not chosen: this detector reads the chair at
// 0.79-0.87 between 1.7 and 3.2 m and collapses to `bench` below ~1.3 m (U3 report §6-2).

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "go2_patrol_manager/patrol_logic.hpp"

using go2_patrol_manager::bearing;
using go2_patrol_manager::distance;
using go2_patrol_manager::parseWaypoints;
using go2_patrol_manager::Pose2;
using go2_patrol_manager::screenConditionOk;
using go2_patrol_manager::standoffPose;
using go2_patrol_manager::travelYaw;
using go2_patrol_manager::Waypoint;

namespace
{
constexpr double kStandoff = 2.0;   // patrol_manager default (see the node header)
constexpr double kWidth = 640.0;    // measured camera stream
constexpr double kHeight = 480.0;
constexpr double kCentreTol = 0.35;
constexpr double kMinHeight = 0.25;
}  // namespace

TEST(StandoffPose, StandsBetweenTheRobotAndTheTarget)
{
  // Robot walking up the aisle at (-6.0, 1.2); target (chair) at (-6.0, 5.2).
  const auto pose = standoffPose(-6.0, 5.2, -6.0, 1.2, kStandoff, 1.5708);
  EXPECT_NEAR(pose.x, -6.0, 1e-9);
  EXPECT_NEAR(pose.y, 5.2 - kStandoff, 1e-9);          // ON the robot's side, never past it
  EXPECT_NEAR(distance(pose.x, pose.y, -6.0, 5.2), kStandoff, 1e-9);
  EXPECT_LT(distance(pose.x, pose.y, -6.0, 1.2), distance(-6.0, 1.2, -6.0, 5.2));
}

TEST(StandoffPose, FacesTheTargetSoTheApproachEndsLookingAtIt)
{
  // This is the property AR-16/AR-18 make load-bearing: the arrival heading must come out
  // of the drive, because this SUT cannot pivot into it afterwards.
  const auto pose = standoffPose(-6.0, 5.2, -6.0, 1.2, kStandoff, 0.0);
  EXPECT_NEAR(pose.yaw, M_PI / 2.0, 1e-9);  // +y, straight at the target

  const auto from_side = standoffPose(-6.0, 5.2, -2.0, 5.2, kStandoff, 0.0);
  EXPECT_NEAR(from_side.yaw, M_PI, 1e-9);   // approached from +x -> looks back along -x
  EXPECT_NEAR(from_side.x, -6.0 + kStandoff, 1e-9);
  EXPECT_NEAR(from_side.y, 5.2, 1e-9);
}

TEST(StandoffPose, LandsOnTheScenarioGoal)
{
  // The scenario's goal IS the standoff point (2.0 m in front of the target), so a robot
  // that stops where this function says lands on the verdict anchor. Asserted here so a
  // standoff change cannot silently walk away from the oracles' radii.
  const auto pose = standoffPose(-6.0, 5.2, -6.0, 1.2, kStandoff, 0.0);
  EXPECT_NEAR(pose.x, -6.0, 1e-9);
  EXPECT_NEAR(pose.y, 3.2, 1e-9);  // = the TB goal
}

TEST(StandoffPose, DegenerateRobotOnTopOfTargetKeepsTheCurrentHeading)
{
  const auto pose = standoffPose(-6.0, 5.2, -6.0, 5.2, kStandoff, 1.234);
  EXPECT_NEAR(pose.yaw, 1.234, 1e-12);
  EXPECT_NEAR(pose.x, -6.0, 1e-12);
}

TEST(ScreenCondition, PassesForACentredChairAtTheStandoff)
{
  // MEASURED box heights for the chair: 169 px @ 2.15 m, 213 px @ 1.74 m. At the 2.0 m
  // standoff the box is ~180 px = 0.37 of the frame.
  EXPECT_TRUE(screenConditionOk(320.0, 180.0, kWidth, kHeight, kCentreTol, kMinHeight));
}

TEST(ScreenCondition, RejectsATargetThatIsTooSmallInFrame)
{
  // MEASURED: the same chair is a 64 px box at 5.5 m — seen, but not "large" (AR-24), and
  // a robot that "held" it from there did not walk up to anything.
  EXPECT_FALSE(screenConditionOk(320.0, 64.0, kWidth, kHeight, kCentreTol, kMinHeight));
}

TEST(ScreenCondition, RejectsATargetOffToTheSide)
{
  // 0.35 * 640 = 224 px of allowed centre error.
  EXPECT_TRUE(screenConditionOk(320.0 + 223.0, 180.0, kWidth, kHeight, kCentreTol, kMinHeight));
  EXPECT_FALSE(screenConditionOk(320.0 + 225.0, 180.0, kWidth, kHeight, kCentreTol, kMinHeight));
  EXPECT_FALSE(screenConditionOk(320.0 - 300.0, 180.0, kWidth, kHeight, kCentreTol, kMinHeight));
}

TEST(ScreenCondition, PassesForAPersonWhoFillsMostOfTheFrame)
{
  // A 1.73 m person at the 2.0 m standoff spans ~317 px (0.66 of the frame) — the same
  // condition, a very different target, no per-class threshold.
  EXPECT_TRUE(screenConditionOk(320.0, 317.0, kWidth, kHeight, kCentreTol, kMinHeight));
}

TEST(ScreenCondition, RejectsDegenerateGeometryInsteadOfDividingByZero)
{
  EXPECT_FALSE(screenConditionOk(320.0, 180.0, 0.0, kHeight, kCentreTol, kMinHeight));
  EXPECT_FALSE(screenConditionOk(320.0, 180.0, kWidth, 0.0, kCentreTol, kMinHeight));
  EXPECT_FALSE(screenConditionOk(320.0, 0.0, kWidth, kHeight, kCentreTol, kMinHeight));
}

TEST(ParseWaypoints, AcceptsTheDefaultRouteAndPreservesOrder)
{
  std::vector<Waypoint> route;
  ASSERT_TRUE(parseWaypoints({-5.5, 0.8, -6.0, 1.2}, route));
  ASSERT_EQ(route.size(), 2U);
  EXPECT_NEAR(route[0].x, -5.5, 1e-12);
  EXPECT_NEAR(route[0].y, 0.8, 1e-12);
  EXPECT_NEAR(route[1].x, -6.0, 1e-12);
  EXPECT_NEAR(route[1].y, 1.2, 1e-12);
}

TEST(ParseWaypoints, RejectsAHalfWrittenRoute)
{
  std::vector<Waypoint> route{Waypoint{9.0, 9.0}};
  EXPECT_FALSE(parseWaypoints({-5.5, 0.8, -6.0}, route));
  EXPECT_FALSE(parseWaypoints({}, route));
  ASSERT_EQ(route.size(), 1U);  // untouched: the caller's route is not half-overwritten
  EXPECT_NEAR(route[0].x, 9.0, 1e-12);
}

TEST(ParseWaypoints, KeepsTheDefaultRouteAwayFromTheVerdictGoal)
{
  // A perception-dead run must FAIL, not coast to a pass by walking the patrol route: the
  // route's closest waypoint has to stay outside the oracles' radii around (-6.0, 3.2).
  std::vector<Waypoint> route;
  ASSERT_TRUE(parseWaypoints({-5.5, 0.8, -6.0, 1.2}, route));
  for (const auto & wp : route) {
    EXPECT_GE(distance(wp.x, wp.y, -6.0, 3.2), 2.0);
  }
}

TEST(TravelYaw, PointsAlongTheDirectionOfTravel)
{
  EXPECT_NEAR(travelYaw(-6.0, 0.0, Waypoint{-6.0, 1.2}, 0.0), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(travelYaw(-5.5, 0.8, Waypoint{-6.0, 1.2}, 0.0), bearing(-5.5, 0.8, -6.0, 1.2), 1e-12);
}

TEST(TravelYaw, KeepsTheCurrentHeadingWhenAlreadyThere)
{
  // Never synthesise a heading that would have to be reached by turning on the spot.
  EXPECT_NEAR(travelYaw(-6.0, 1.2, Waypoint{-6.0, 1.2}, 1.5708), 1.5708, 1e-12);
}
