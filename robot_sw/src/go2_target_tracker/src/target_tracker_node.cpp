// target_tracker_node.cpp — 2D detections + depth + TF  ->  confirmed map-frame targets.
//
//   /detections               (vision_msgs/Detection2DArray, from go2_detector)
//   /camera/depth/image_raw   (sensor_msgs/Image, 32FC1)
//   /camera/camera_info       (sensor_msgs/CameraInfo)
//   TF go2_camera -> map
//        |
//        v
//   /targets                  (vision_msgs/Detection3DArray, frame `map`, class-labeled
//                              CONFIRMED tracks)
//
// Layer rule: the detector below does not know the mission and the manager above does not
// know pixels. This node is the only place where a pixel becomes a place. It publishes
// CONFIRMED tracks only — a target that has been seen `min_hits` times at the same spot —
// because one frame's confidence is not evidence on this robot: the same chair measures
// 0.22..0.80 depending on whether it is whole and large in the frame. The time filter is
// what turns that into a stable claim, and it is why the manager can treat a matching
// /targets entry as "go".
//
// Each published detection carries its CLASS, so the manager can be told "find a person"
// and ignore the chair standing next to it.
//
// Local-first: plain ROS 2. Point it at a rosbag, a simulator or a real camera.

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "go2_target_tracker/projection.hpp"

namespace go2_target_tracker
{

class TargetTracker : public rclcpp::Node
{
public:
  TargetTracker()
  : Node("go2_target_tracker")
  {
    // --- parameters (every one declared, with the measurement behind its default) ---
    const auto detections_topic = declare_parameter<std::string>("detections_topic", "/detections");
    const auto depth_topic =
      declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    const auto camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    const auto targets_topic = declare_parameter<std::string>("targets_topic", "/targets");
    // The patrol targets. Both are COCO classes yolo11n emits directly; scenery
    // (box/desk/forklift) is not in this list, so a forklift measured as `truck 0.75` can
    // never become a target.
    // ⚠ go2_patrol_manager declares the SAME parameter with the SAME default and uses it
    // to validate incoming goals. Override one without the other and a goal class it
    // accepts can never be published here. Change both, or neither.
    target_classes_ = declare_parameter<std::vector<std::string>>(
      "target_classes", std::vector<std::string>{"person", "chair"});
    // 0.5 = the target-confidence threshold. Kept here as well so the tracker is still
    // safe when someone runs the detector wide open to look at raw output.
    min_confidence_ = declare_parameter<double>("min_confidence", 0.5);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    // 1.5 m — sized by MEASUREMENT, and the first live run is why it is not 0.75.
    // The projection lands in the MAP frame through AMCL's map->odom, so early in a
    // mission it carries AMCL's belief error: the live chair run first placed the chair
    // at (-4.76, 5.08) and walked it to (-6.28, 5.10) as the filter converged on the
    // real (-6.00, 5.20) — 1.5 m of travel for a target that never moved. At 0.75 m the
    // corrected measurements opened a SECOND track and the manager approached the ghost.
    // The radius must therefore exceed the localisation error, while staying under the
    // separation between a target and the nearest thing that could be mistaken for one
    // (>= 1.65 m in the scene this app was tuned on).
    assoc_radius_m_ = declare_parameter<double>("assoc_radius_m", 1.5);
    // 3 hits at the detector's 5 Hz = 0.6 s of sim time agreeing with itself.
    min_hits_ = declare_parameter<int>("min_hits", 3);
    position_lpf_alpha_ = declare_parameter<double>("position_lpf_alpha", 0.4);
    // 2 -> a 5x5 px window. The chair's bbox centre often falls in the gap between seat
    // and backrest; the median of 25 samples ignores that minority (see projection.hpp).
    depth_window_half_px_ = declare_parameter<int>("depth_window_half_px", 2);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.3);
    // 8 m: past that the target is a handful of pixels and the projection error grows
    // with the square of range. Detections beyond it are dropped, not projected badly.
    max_depth_m_ = declare_parameter<double>("max_depth_m", 8.0);
    // depth and rgb are published at the same 10 Hz rate from the same frame (measured on
    // this sim), so 0.25 s = two frames of slack, not a guess. With the buffer below the
    // usual match is EXACT (same stamp); this bound is what rejects a detection whose
    // depth frame never arrived.
    max_depth_age_s_ = declare_parameter<double>("max_depth_age_s", 0.25);
    // 10 frames = 1.0 s of sim time at the measured 10 Hz depth rate: longer than the
    // worst detector latency observed live (0.4 s of sim skew), short enough to stay a
    // handful of megabytes.
    depth_buffer_size_ =
      std::max(1, static_cast<int>(declare_parameter<int>("depth_buffer_size", 10)));
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.1);
    // Unconfirmed tracks decay; CONFIRMED ones do not (see prune()).
    track_timeout_s_ = declare_parameter<double>("track_timeout_s", 5.0);
    log_period_s_ = declare_parameter<double>("log_period_s", 10.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // BEST_EFFORT for the sensor streams: a BEST_EFFORT subscription matches both a
    // RELIABLE publisher (what a sim bridge typically uses) and a BEST_EFFORT one (what a
    // real camera driver uses); the reverse does not hold.
    //
    // ⚠ Depth keeps a short HISTORY, not just the latest frame. Detections arrive LATE
    // (the detector spends 25-500 ms of CPU per frame), so by the time one lands here the
    // newest depth frame is already 0.1-0.4 s of sim time ahead of the image the boxes
    // came from — measured live on the first run: "depth frame is 0.40 s away from the
    // detections". Back-projecting a bbox with a depth frame taken from a different pose
    // is how a target ends up inside a shelf. The buffer lets us pick the depth frame
    // that actually belongs to the image (usually the exact same stamp).
    const auto depth_qos = rclcpp::QoS(static_cast<size_t>(depth_buffer_size_)).best_effort();
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, depth_qos, [this](sensor_msgs::msg::Image::SharedPtr msg) {
        depth_buffer_.push_back(std::move(msg));
        while (static_cast<int>(depth_buffer_.size()) > depth_buffer_size_) {
          depth_buffer_.pop_front();
        }
      });
    const auto sensor_qos = rclcpp::QoS(1).best_effort();
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        intrinsics_ = CameraIntrinsics{msg->k[0], msg->k[4], msg->k[2], msg->k[5]};
      });
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_topic, rclcpp::QoS(5).reliable(),
      [this](vision_msgs::msg::Detection2DArray::SharedPtr msg) { onDetections(*msg); });
    targets_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      targets_topic, rclcpp::QoS(5).reliable());

    std::string classes;
    for (const auto & c : target_classes_) {
      classes += (classes.empty() ? "" : ",") + c;
    }
    RCLCPP_INFO(
      get_logger(),
      "go2_target_tracker up: %s + %s + %s -> %s | classes=[%s] conf>=%.2f min_hits=%d "
      "assoc=%.2f m depth=[%.2f,%.2f] m",
      detections_topic.c_str(), depth_topic.c_str(), camera_info_topic.c_str(),
      targets_topic.c_str(), classes.c_str(), min_confidence_, min_hits_, assoc_radius_m_,
      min_depth_m_, max_depth_m_);
  }

private:
  static double stampSeconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  bool isTargetClass(const std::string & class_id) const
  {
    return std::find(target_classes_.begin(), target_classes_.end(), class_id) !=
           target_classes_.end();
  }

  /// Transform a point from the detection's own frame into `map_frame_`.
  ///
  /// Tries the detection's stamp first (the honest answer: where the target was when the
  /// pixel was taken) and falls back to the LATEST transform when tf2 cannot interpolate
  /// yet. The fallback is safe for this mission because the targets in it are static and
  /// the robot moves ~0.4 m/s, i.e. a 0.1 s TF age is a 4 cm error against a 0.75 m
  /// association radius. It is logged, so
  /// "we quietly used a stale transform" is never invisible.
  bool toMap(const geometry_msgs::msg::PointStamped & in, geometry_msgs::msg::PointStamped & out)
  {
    try {
      out = tf_buffer_->transform(in, map_frame_, tf2::durationFromSec(tf_timeout_s_));
      return true;
    } catch (const tf2::TransformException & exc) {
      geometry_msgs::msg::PointStamped latest = in;
      latest.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      try {
        out = tf_buffer_->transform(latest, map_frame_, tf2::durationFromSec(tf_timeout_s_));
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "TF at the detection stamp unavailable (%s) — using the latest transform instead",
          exc.what());
        return true;
      } catch (const tf2::TransformException & exc2) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "dropping detections, no TF to %s: %s",
          map_frame_.c_str(), exc2.what());
        return false;
      }
    }
  }

  void onDetections(const vision_msgs::msg::Detection2DArray & msg)
  {
    const double stamp_s = stampSeconds(msg.header.stamp);
    if (!intrinsics_.valid()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no camera_info yet — cannot project detections");
      return;
    }
    if (depth_buffer_.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no depth frame yet — cannot project detections");
      return;
    }
    // The depth frame CLOSEST IN SIM TIME to the image these boxes came from.
    sensor_msgs::msg::Image::SharedPtr depth;
    double depth_age = 0.0;
    for (const auto & candidate : depth_buffer_) {
      const double age = std::abs(stamp_s - stampSeconds(candidate->header.stamp));
      if (depth == nullptr || age < depth_age) {
        depth = candidate;
        depth_age = age;
      }
    }
    if (depth->encoding != "32FC1") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "depth encoding %s is not supported (32FC1 expected) — detections are not projected",
        depth->encoding.c_str());
      return;
    }
    if (depth_age > max_depth_age_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "closest depth frame is %.2f s away from the detections (limit %.2f s) — skipping",
        depth_age, max_depth_age_s_);
      return;
    }

    for (const auto & det : msg.detections) {
      if (det.results.empty()) {
        continue;
      }
      const auto & hypothesis = det.results.front().hypothesis;
      if (!isTargetClass(hypothesis.class_id) || hypothesis.score < min_confidence_) {
        continue;
      }
      const int u = static_cast<int>(std::lround(det.bbox.center.position.x));
      const int v = static_cast<int>(std::lround(det.bbox.center.position.y));
      const double depth_m = medianDepth(
        depth->data.data(), depth->data.size(), static_cast<int>(depth->width),
        static_cast<int>(depth->height), static_cast<int>(depth->step), u, v,
        depth_window_half_px_, min_depth_m_, max_depth_m_);
      if (!std::isfinite(depth_m)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "no usable depth under the %s bbox centre (%d, %d) — detection dropped",
          hypothesis.class_id.c_str(), u, v);
        continue;
      }

      const Point3 camera_point = backProject(
        det.bbox.center.position.x, det.bbox.center.position.y, depth_m, intrinsics_);
      geometry_msgs::msg::PointStamped in;
      in.header = msg.header;  // the detection carries the IMAGE header: sim stamp + optical frame
      in.point.x = camera_point.x;
      in.point.y = camera_point.y;
      in.point.z = camera_point.z;
      geometry_msgs::msg::PointStamped out;
      if (!toMap(in, out)) {
        continue;
      }
      integrate(hypothesis.class_id, Point3{out.point.x, out.point.y, out.point.z}, stamp_s);
    }

    prune(stamp_s);
    publish(msg.header.stamp);
    logPeriodically(stamp_s);
  }

  void integrate(const std::string & class_id, const Point3 & position, double stamp_s)
  {
    const int index = nearestTrack(tracks_, position, class_id, assoc_radius_m_);
    if (index < 0) {
      tracks_.push_back(Track{class_id, position, 1, stamp_s, false, false});
      return;
    }
    Track & track = tracks_[static_cast<std::size_t>(index)];
    track.position = blend(track.position, position, position_lpf_alpha_);
    track.hits += 1;
    track.last_seen_s = stamp_s;
    if (!track.confirmed && track.hits >= min_hits_) {
      track.confirmed = true;
    }
    if (track.confirmed && !track.announced) {
      track.announced = true;
      RCLCPP_INFO(
        get_logger(), "target CONFIRMED: %s at map (%.2f, %.2f, %.2f) after %d hits",
        track.class_id.c_str(), track.position.x, track.position.y, track.position.z,
        track.hits);
    }
  }

  /// Drop UNCONFIRMED tracks that went quiet. Confirmed ones are kept on purpose: the
  /// targets in this mission are static, and the manager must not lose its goal while the
  /// robot turns the last corner and the target slides out of the frame for a moment.
  void prune(double now_s)
  {
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [&](const Track & t) {
          return !t.confirmed && (now_s - t.last_seen_s) > track_timeout_s_;
        }),
      tracks_.end());
  }

  void publish(const builtin_interfaces::msg::Time & stamp)
  {
    vision_msgs::msg::Detection3DArray out;
    out.header.stamp = stamp;  // the SOURCE image stamp: same instant as the pixels
    out.header.frame_id = map_frame_;
    for (const auto & track : tracks_) {
      if (!track.confirmed) {
        continue;
      }
      vision_msgs::msg::Detection3D det;
      det.header = out.header;
      det.id = track.class_id;  // same convention as Detection2D.id upstream: the class name
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = track.class_id;
      // score = the time-filtered confirmation claim (min_hits agreeing frames), NOT a raw
      // model confidence — single-frame confidence is exactly what this node distrusts.
      hyp.hypothesis.score = 1.0;
      hyp.pose.pose.position.x = track.position.x;
      hyp.pose.pose.position.y = track.position.y;
      hyp.pose.pose.position.z = track.position.z;
      hyp.pose.pose.orientation.w = 1.0;  // position-only target: no orientation is claimed
      det.results.push_back(hyp);
      // det.bbox stays zeroed: no 3D extent is measured, so none is claimed.
      out.detections.push_back(det);
    }
    // Published on EVERY processed frame, empty included: "nothing confirmed yet" is a
    // fact the manager needs, and a topic that only speaks on success is a topic whose
    // silence is ambiguous.
    targets_pub_->publish(out);
  }

  void logPeriodically(double now_s)
  {
    if (log_period_s_ <= 0.0) {
      return;
    }
    if (last_log_s_ > 0.0 && (now_s - last_log_s_) < log_period_s_) {
      return;
    }
    last_log_s_ = now_s;
    std::size_t confirmed = 0;
    for (const auto & track : tracks_) {
      confirmed += track.confirmed ? 1 : 0;
    }
    RCLCPP_INFO(
      get_logger(), "tracks: %zu total, %zu confirmed (sim_time=%.2f s)", tracks_.size(),
      confirmed, now_s);
  }

  std::vector<std::string> target_classes_;
  double min_confidence_{0.5};
  std::string map_frame_;
  double assoc_radius_m_{0.75};
  int min_hits_{3};
  double position_lpf_alpha_{0.4};
  int depth_window_half_px_{2};
  double min_depth_m_{0.3};
  double max_depth_m_{8.0};
  double max_depth_age_s_{0.25};
  int depth_buffer_size_{10};
  double tf_timeout_s_{0.1};
  double track_timeout_s_{5.0};
  double log_period_s_{10.0};
  double last_log_s_{0.0};

  CameraIntrinsics intrinsics_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> depth_buffer_;
  std::vector<Track> tracks_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr targets_pub_;
};

}  // namespace go2_target_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_target_tracker::TargetTracker>());
  rclcpp::shutdown();
  return 0;
}
