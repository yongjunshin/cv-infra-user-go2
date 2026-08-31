// CPU unit tests for the tracker's pure math (no ROS, no sim, no model).
//
// The numbers below are the MEASURED ones this SUT actually runs against:
// k = [366.4996, 0, 320, 0, 366.4997, 240] on a 640x480 32FC1 depth frame with step 2560
// (platform C3 §4-3), a 0.877 m chair and a 1.73 m person (C0/W0 asset census).

#include <cmath>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "go2_target_tracker/projection.hpp"

using go2_target_tracker::backProject;
using go2_target_tracker::blend;
using go2_target_tracker::CameraIntrinsics;
using go2_target_tracker::medianDepth;
using go2_target_tracker::nearestTrack;
using go2_target_tracker::Point3;
using go2_target_tracker::Track;

namespace
{
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr int kStep = kWidth * static_cast<int>(sizeof(float));  // 2560, measured

CameraIntrinsics measuredK() { return CameraIntrinsics{366.4996, 366.4997, 320.0, 240.0}; }

/// A 640x480 32FC1 frame filled with `fill` metres.
std::vector<uint8_t> depthFrame(float fill)
{
  std::vector<uint8_t> data(static_cast<std::size_t>(kStep) * kHeight);
  for (int row = 0; row < kHeight; ++row) {
    for (int col = 0; col < kWidth; ++col) {
      std::memcpy(
        data.data() + static_cast<std::size_t>(row) * kStep + col * sizeof(float), &fill,
        sizeof(float));
    }
  }
  return data;
}

void setPixel(std::vector<uint8_t> & data, int u, int v, float value)
{
  std::memcpy(
    data.data() + static_cast<std::size_t>(v) * kStep + u * sizeof(float), &value, sizeof(float));
}
}  // namespace

TEST(BackProject, PrincipalPointProjectsOnTheOpticalAxis)
{
  const auto p = backProject(320.0, 240.0, 1.2, measuredK());
  EXPECT_NEAR(p.x, 0.0, 1e-12);
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(p.z, 1.2);
}

TEST(BackProject, OpticalFrameSignsAreRightAndDown)
{
  // REP-103 optical frame: +x right, +y down, +z forward. A pixel right of and below the
  // principal point must therefore land at positive x and positive y.
  const auto p = backProject(420.0, 300.0, 2.0, measuredK());
  EXPECT_GT(p.x, 0.0);
  EXPECT_GT(p.y, 0.0);
  EXPECT_NEAR(p.x, (420.0 - 320.0) * 2.0 / 366.4996, 1e-9);
  EXPECT_NEAR(p.y, (300.0 - 240.0) * 2.0 / 366.4997, 1e-9);
}

TEST(BackProject, RangeScalesLinearlyWithDepth)
{
  const auto near_p = backProject(420.0, 240.0, 1.0, measuredK());
  const auto far_p = backProject(420.0, 240.0, 3.0, measuredK());
  EXPECT_NEAR(far_p.x, 3.0 * near_p.x, 1e-9);
}

TEST(BackProject, MatchesTheChairStandoffGeometryTheScenariosUse)
{
  // The TB scenarios put the target 1.2 m in front of the robot (standoff). A chair whose
  // 0.877 m body then spans ~268 px is the arithmetic in the scenario headers; here we
  // only assert the inverse direction the tracker uses: a centred bbox at 1.2 m is 1.2 m
  // straight ahead of the camera, not somewhere off to the side.
  const auto p = backProject(320.0, 300.0, 1.2, measuredK());
  EXPECT_NEAR(p.x, 0.0, 1e-12);
  EXPECT_NEAR(std::hypot(p.x, p.z), 1.2, 1e-9);
}

TEST(MedianDepth, IgnoresASingleFarOutlierUnderTheBboxCentre)
{
  // The failure this defends against: the chair's bbox centre falls in the gap between
  // seat and backrest and reads the wall 8 m behind it.
  auto data = depthFrame(1.20F);
  setPixel(data, 320, 240, 7.9F);
  const double d =
    medianDepth(data.data(), data.size(), kWidth, kHeight, kStep, 320, 240, 2, 0.3, 8.0);
  EXPECT_NEAR(d, 1.20, 1e-6);
}

TEST(MedianDepth, RejectsNonFiniteAndOutOfRangeSamples)
{
  auto data = depthFrame(std::numeric_limits<float>::quiet_NaN());
  const double all_nan =
    medianDepth(data.data(), data.size(), kWidth, kHeight, kStep, 320, 240, 2, 0.3, 8.0);
  EXPECT_TRUE(std::isnan(all_nan));

  auto too_far = depthFrame(30.0F);  // beyond max_depth_m
  EXPECT_TRUE(
    std::isnan(medianDepth(too_far.data(), too_far.size(), kWidth, kHeight, kStep, 320, 240, 2,
                           0.3, 8.0)));

  auto too_near = depthFrame(0.05F);  // inside the camera's own near clip
  EXPECT_TRUE(
    std::isnan(medianDepth(too_near.data(), too_near.size(), kWidth, kHeight, kStep, 320, 240, 2,
                           0.3, 8.0)));
}

TEST(MedianDepth, SurvivesAWindowClippedByTheFrameEdge)
{
  auto data = depthFrame(2.5F);
  const double corner =
    medianDepth(data.data(), data.size(), kWidth, kHeight, kStep, 0, 0, 2, 0.3, 8.0);
  EXPECT_NEAR(corner, 2.5, 1e-6);
  const double far_corner = medianDepth(
    data.data(), data.size(), kWidth, kHeight, kStep, kWidth - 1, kHeight - 1, 2, 0.3, 8.0);
  EXPECT_NEAR(far_corner, 2.5, 1e-6);
}

TEST(MedianDepth, RejectsPixelsOutsideTheFrameAndNullBuffers)
{
  auto data = depthFrame(2.5F);
  EXPECT_TRUE(
    std::isnan(medianDepth(data.data(), data.size(), kWidth, kHeight, kStep, -1, 10, 2, 0.3, 8.0)));
  EXPECT_TRUE(std::isnan(
    medianDepth(data.data(), data.size(), kWidth, kHeight, kStep, kWidth, 10, 2, 0.3, 8.0)));
  EXPECT_TRUE(std::isnan(medianDepth(nullptr, 0, kWidth, kHeight, kStep, 10, 10, 2, 0.3, 8.0)));
}

TEST(MedianDepth, NeverReadsPastATruncatedMessage)
{
  // A short `data` vector must degrade to NaN, not to a read out of bounds.
  auto data = depthFrame(1.0F);
  data.resize(static_cast<std::size_t>(kStep) * 10);  // only 10 rows survive
  EXPECT_TRUE(
    std::isnan(medianDepth(data.data(), data.size(), kWidth, kHeight, kStep, 320, 470, 2, 0.3,
                           8.0)));
}

TEST(MedianDepth, ReadsTheRowStrideNotTheWidth)
{
  // Guard against the classic "width * 4 == step" assumption: a padded frame whose step
  // is larger than width*4 must still address the right row.
  const int padded_step = kStep + 64;
  std::vector<uint8_t> data(static_cast<std::size_t>(padded_step) * kHeight, 0);
  const float value = 3.5F;
  for (int row = 0; row < kHeight; ++row) {
    for (int col = 0; col < kWidth; ++col) {
      std::memcpy(
        data.data() + static_cast<std::size_t>(row) * padded_step + col * sizeof(float), &value,
        sizeof(float));
    }
  }
  const double d =
    medianDepth(data.data(), data.size(), kWidth, kHeight, padded_step, 100, 400, 2, 0.3, 8.0);
  EXPECT_NEAR(d, 3.5, 1e-6);
}

TEST(NearestTrack, AssociatesOnlyWithinTheRadius)
{
  std::vector<Track> tracks{Track{"chair", Point3{-6.0, 5.2, 0.4}, 3, 1.0, true, true}};
  EXPECT_EQ(nearestTrack(tracks, Point3{-6.1, 5.3, 0.45}, "chair", 0.75), 0);
  EXPECT_EQ(nearestTrack(tracks, Point3{-6.0, 3.0, 0.4}, "chair", 0.75), -1);
}

TEST(NearestTrack, NeverAssociatesAcrossClasses)
{
  // A person standing next to the chair must not swallow the chair's measurements.
  std::vector<Track> tracks{Track{"person", Point3{-6.0, 5.2, 0.9}, 5, 1.0, true, true}};
  EXPECT_EQ(nearestTrack(tracks, Point3{-6.0, 5.2, 0.9}, "chair", 0.75), -1);
}

TEST(NearestTrack, PicksTheNearestOfSeveral)
{
  std::vector<Track> tracks{
    Track{"chair", Point3{-6.0, 5.2, 0.4}, 3, 1.0, true, true},
    Track{"chair", Point3{-6.0, 5.6, 0.4}, 3, 1.0, true, true}};
  EXPECT_EQ(nearestTrack(tracks, Point3{-6.0, 5.55, 0.4}, "chair", 0.75), 1);
}

TEST(Blend, EndpointsAndMidpoint)
{
  const Point3 held{0.0, 0.0, 0.0};
  const Point3 fresh{2.0, 4.0, 6.0};
  EXPECT_NEAR(blend(held, fresh, 0.0).x, 0.0, 1e-12);
  EXPECT_NEAR(blend(held, fresh, 1.0).y, 4.0, 1e-12);
  EXPECT_NEAR(blend(held, fresh, 0.5).z, 3.0, 1e-12);
}

TEST(Blend, ClampsAnAbsurdAlphaInsteadOfExtrapolating)
{
  const Point3 held{1.0, 1.0, 1.0};
  const Point3 fresh{2.0, 2.0, 2.0};
  EXPECT_NEAR(blend(held, fresh, 5.0).x, 2.0, 1e-12);
  EXPECT_NEAR(blend(held, fresh, -5.0).x, 1.0, 1e-12);
}
