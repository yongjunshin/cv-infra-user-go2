// projection.hpp — the tracker's PURE math: pixel + depth -> 3D, and track association.
//
// Nothing in here knows about ROS, tf2 or the mission. That is deliberate: these four
// functions are the ones that can be silently wrong (a swapped fx/fy, an off-by-one row
// stride, a depth window that averages the wall behind the chair, an association radius
// applied across classes) and stay wrong for a whole live run before anyone notices. They
// are therefore compiled and asserted on the CPU (test/test_projection.cpp) with no ROS
// in the loop — the same split `go2_detector/detection_logic.py` uses on the Python side.
//
// Frame convention (MEASURED): the image is stamped in `go2_camera`,
// which is a ROS **optical** frame (REP-103: x right, y down, z forward), so a
// back-projected point is already expressed in that frame and tf2 alone carries it to
// `map`. There is no separate `camera_link` and no axis permutation to apply here.

#ifndef GO2_TARGET_TRACKER__PROJECTION_HPP_
#define GO2_TARGET_TRACKER__PROJECTION_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace go2_target_tracker
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// Pinhole intrinsics as they arrive in `sensor_msgs/CameraInfo.k`
/// (MEASURED on this sim: k = [366.4996, 0, 320, 0, 366.4997, 240, 0, 0, 1], 640x480,
/// cross-checked against the vendor intrinsics).
struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  bool valid() const { return fx > 0.0 && fy > 0.0; }
};

/// Back-project pixel (u, v) at `depth_m` into the camera OPTICAL frame.
///
/// ASSUMPTION, surfaced not hidden: `depth_m` is Z-depth (distance to the image plane),
/// which is what this depth stream carries (32FC1, `distance_to_image_plane`
/// family). If it were radial distance instead, this over-estimates the range by
/// 1/cos(angle) — 1.6 % at the frame corner, 0 % at the centre where a bbox centre of a
/// centred target sits. The tracker only ever projects bbox CENTRES, so the residual is
/// below the association radius by two orders of magnitude either way.
inline Point3 backProject(double u, double v, double depth_m, const CameraIntrinsics & k)
{
  return Point3{(u - k.cx) * depth_m / k.fx, (v - k.cy) * depth_m / k.fy, depth_m};
}

/// Median of the FINITE, in-range depth samples in a (2*half+1)^2 window around (u, v).
///
/// Why a median and not the centre pixel: the bbox centre of a chair frequently lands on
/// a gap between the seat and the backrest, where the depth is the wall 8 m behind it.
/// One such sample would place the target inside a shelf. The median of a small window is
/// the cheapest estimator that ignores that minority — and it is exactly why this function
/// returns NaN (not 0.0) when the window holds nothing usable: a 0.0 depth would project
/// the target onto the camera itself, which reads as "found it, right here".
///
/// `data`/`step` are the raw `sensor_msgs/Image` bytes of a 32FC1 frame (step = row bytes,
/// MEASURED 2560 for 640 px). Bounds are clamped, so a bbox centre on the frame edge is
/// handled by shrinking the window, never by reading out of the buffer.
inline double medianDepth(
  const uint8_t * data, std::size_t size, int width, int height, int step, int u, int v,
  int half, double min_depth_m, double max_depth_m)
{
  const double nan = std::nan("");
  if (data == nullptr || width <= 0 || height <= 0 || step <= 0) {
    return nan;
  }
  if (u < 0 || v < 0 || u >= width || v >= height) {
    return nan;
  }
  const int u0 = std::max(0, u - half);
  const int u1 = std::min(width - 1, u + half);
  const int v0 = std::max(0, v - half);
  const int v1 = std::min(height - 1, v + half);

  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>((u1 - u0 + 1) * (v1 - v0 + 1)));
  for (int row = v0; row <= v1; ++row) {
    for (int col = u0; col <= u1; ++col) {
      const std::size_t offset =
        static_cast<std::size_t>(row) * static_cast<std::size_t>(step) +
        static_cast<std::size_t>(col) * sizeof(float);
      if (offset + sizeof(float) > size) {
        continue;  // truncated message: skip rather than read past the buffer
      }
      float value = 0.0F;
      std::memcpy(&value, data + offset, sizeof(float));
      const double d = static_cast<double>(value);
      if (std::isfinite(d) && d >= min_depth_m && d <= max_depth_m) {
        samples.push_back(d);
      }
    }
  }
  if (samples.empty()) {
    return nan;
  }
  const std::size_t mid = samples.size() / 2;
  std::nth_element(samples.begin(), samples.begin() + static_cast<long>(mid), samples.end());
  return samples[mid];
}

/// One tracked target in the map frame. `hits` is the TIME filter this app needs:
/// a single frame's confidence swings 0.22..0.80 on the same chair depending on framing,
/// so "seen N times at the same place" is the claim we publish, not "seen once".
struct Track
{
  std::string class_id;
  Point3 position;
  int hits{0};
  double last_seen_s{0.0};
  bool confirmed{false};
  bool announced{false};
};

/// Index of the nearest SAME-CLASS track within `radius_m`, or -1.
///
/// Class is part of the predicate on purpose: a person standing next to a chair must not
/// absorb the chair's measurements just because it is closer than the radius.
inline int nearestTrack(
  const std::vector<Track> & tracks, const Point3 & p, const std::string & class_id,
  double radius_m)
{
  int best = -1;
  double best_d2 = radius_m * radius_m;
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    if (tracks[i].class_id != class_id) {
      continue;
    }
    const double dx = tracks[i].position.x - p.x;
    const double dy = tracks[i].position.y - p.y;
    const double dz = tracks[i].position.z - p.z;
    const double d2 = dx * dx + dy * dy + dz * dz;
    if (d2 <= best_d2) {
      best_d2 = d2;
      best = static_cast<int>(i);
    }
  }
  return best;
}

/// Low-pass a track position toward a new measurement (`alpha` = weight of the NEW one).
inline Point3 blend(const Point3 & held, const Point3 & fresh, double alpha)
{
  const double a = std::clamp(alpha, 0.0, 1.0);
  return Point3{
    held.x + a * (fresh.x - held.x), held.y + a * (fresh.y - held.y),
    held.z + a * (fresh.z - held.z)};
}

}  // namespace go2_target_tracker

#endif  // GO2_TARGET_TRACKER__PROJECTION_HPP_
