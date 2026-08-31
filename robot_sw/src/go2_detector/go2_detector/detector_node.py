"""go2_detector — a thin YOLO11n CPU detector node. It does not know the mission.

    /camera/image_raw  (sensor_msgs/Image)  ->  /detections (vision_msgs/Detection2DArray)

Design rules it obeys (patrol app DESIGN.md, master plan §1-4/§1-7):
  * **Publish what it sees.** No target/class decision here — the class filter is empty
    by default. Deciding "is that the patrol target?" is the tracker/manager's job (U3),
    so this node stays reusable and testable on its own.
  * **CPU only.** The weights are sealed into the SUT image at build time (decision
    2026-08-31 D1-A) and inference runs on CPU; the platform never gives this container a
    GPU (D1-P2). `torch_threads` caps how much of the CI host's CPU it takes.
  * **Latest frame only, at a fixed rate.** Subscription depth is 1 and the throttle is
    keyed on the IMAGE stamp (= sim time), so a slow wall clock cannot turn into a
    growing queue of stale frames, and the detector's duty cycle in sim time is the same
    on a fast and a slow machine.

Runs anywhere a `sensor_msgs/Image` shows up — cv-infra's runner, the platform's
dev-world, a rosbag, or a real camera (master plan §1-8, local-first development).
"""

import os
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from go2_detector.detection_logic import (
    image_to_bgr,
    select_detections,
    should_process,
    stamp_to_seconds,
)

#: Where the Dockerfile seals the weights (build-time fetch + sha256 verify).
DEFAULT_MODEL_PATH = "/opt/models/yolo11n.pt"


class Go2Detector(Node):
    def __init__(self):
        super().__init__("go2_detector")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("process_rate_hz", 5.0)
        self.declare_parameter("conf_threshold", 0.25)
        # dynamic_typing, not a STRING_ARRAY descriptor: rclpy infers the type from the
        # DEFAULT, and an empty list is inferred as BYTE_ARRAY, so
        # `-p class_whitelist:="['person']"` is then rejected at startup with
        # InvalidParameterTypeException (measured 2026-09-01 — the node died on launch).
        # Declaring the type without a default instead makes the unset parameter raise
        # ParameterUninitializedException on read. dynamic typing keeps both the empty
        # default and the string-list override working.
        self.declare_parameter(
            "class_whitelist",
            [],
            ParameterDescriptor(
                dynamic_typing=True,
                description="COCO class names to publish. EMPTY (default) = publish all.",
            ),
        )
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("torch_threads", 2)
        self.declare_parameter("log_period_s", 10.0)

        get = self.get_parameter
        image_topic = get("image_topic").value
        detections_topic = get("detections_topic").value
        model_path = get("model_path").value
        rate_hz = float(get("process_rate_hz").value)
        self._min_period_s = 1.0 / rate_hz if rate_hz > 0.0 else 0.0
        self._conf_threshold = float(get("conf_threshold").value)
        self._class_whitelist = list(get("class_whitelist").value or [])
        self._imgsz = int(get("imgsz").value)
        self._log_period_s = float(get("log_period_s").value)
        threads = int(get("torch_threads").value)

        self._model = self._load_model(model_path, threads)

        self._last_stamp_s = None
        self._last_log_s = None
        self._processed = 0
        self._published_once = False
        self._decode_warned = False

        # Images: BEST_EFFORT + depth 1. BEST_EFFORT is the compatible choice (a
        # BEST_EFFORT subscription matches both RELIABLE and BEST_EFFORT publishers,
        # not the other way round) — the platform publishes RELIABLE (runner C3 §2),
        # a real camera driver publishes SensorDataQoS. Depth 1 is what makes "latest
        # frame only" true in the middleware instead of only in our throttle.
        self._sub = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._pub = self.create_publisher(
            Detection2DArray,
            detections_topic,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
        )

        self.get_logger().info(
            f"go2_detector up: {image_topic} -> {detections_topic} | model={model_path} "
            f"rate={rate_hz} Hz conf={self._conf_threshold} imgsz={self._imgsz} "
            f"threads={threads} whitelist={self._class_whitelist or 'ALL'}"
        )

    def _load_model(self, model_path, threads):
        """Load the sealed weights. Loud and fatal if they are missing — a detector that
        silently publishes nothing is worse than a container that fails to start."""
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f"model not found: {model_path} (the SUT image seals it at build time; "
                "override with -p model_path:=<path> when running outside the image)"
            )
        # Imported here, not at module import: keeps `detection_logic` unit tests free of
        # torch/ultralytics, and lets the env guards below land before ultralytics reads
        # them. YOLO_AUTOINSTALL=false turns "pip-install a missing dep at runtime" into
        # a loud error — a sealed SUT must never reach out to the network mid-run.
        os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/ultralytics")
        os.environ.setdefault("YOLO_AUTOINSTALL", "false")
        import torch
        from ultralytics import YOLO

        torch.set_num_threads(max(1, threads))
        self.get_logger().info(
            f"torch {torch.__version__} (cuda build={torch.version.cuda}, "
            f"available={torch.cuda.is_available()}) threads<={max(1, threads)}"
        )
        return YOLO(model_path)

    def _on_image(self, msg):
        stamp_s = stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if not should_process(stamp_s, self._last_stamp_s, self._min_period_s):
            return
        self._last_stamp_s = stamp_s

        try:
            frame = image_to_bgr(msg.height, msg.width, msg.step, msg.encoding, msg.data)
        except ValueError as exc:
            if not self._decode_warned:  # once — a bad publisher would flood the log
                self._decode_warned = True
                self.get_logger().warning(f"dropping frames, cannot decode image: {exc}")
            return

        t0 = time.perf_counter()
        result = self._model.predict(
            source=frame, imgsz=self._imgsz, conf=self._conf_threshold, verbose=False
        )[0]
        latency_ms = (time.perf_counter() - t0) * 1e3

        raw = [
            (
                result.names[int(box.cls)],
                float(box.conf),
                tuple(float(v) for v in box.xyxy[0].tolist()),
            )
            for box in result.boxes
        ]
        kept = select_detections(raw, self._conf_threshold, self._class_whitelist)
        self._pub.publish(self._to_msg(msg.header, kept))
        self._processed += 1
        self._log(stamp_s, latency_ms, kept)

    def _to_msg(self, header, kept):
        """Detection2DArray stamped with the SOURCE image's header — same sim time, same
        camera frame (`go2_camera`, a ROS optical frame), so a consumer can back-project
        a box with camera_info and let tf2 do the rest (runner C3 §2-2)."""
        out = Detection2DArray()
        out.header = header
        for name, score, (cx, cy, size_x, size_y) in kept:
            det = Detection2D()
            det.header = header
            det.id = name
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = name
            hypothesis.hypothesis.score = score
            det.results.append(hypothesis)
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = size_x
            det.bbox.size_y = size_y
            out.detections.append(det)
        return out

    def _log(self, stamp_s, latency_ms, kept):
        if not self._published_once:
            self._published_once = True
            self._last_log_s = stamp_s
            self.get_logger().info(
                f"first detections published at sim_time={stamp_s:.3f}s "
                f"({len(kept)} object(s), {latency_ms:.0f} ms CPU)"
            )
            return
        if self._log_period_s > 0.0 and stamp_s - (self._last_log_s or 0.0) >= self._log_period_s:
            self._last_log_s = stamp_s
            names = ",".join(sorted({n for n, _, _ in kept})) or "-"
            self.get_logger().info(
                f"detections: {self._processed} frames processed, last {len(kept)} "
                f"object(s) [{names}] in {latency_ms:.0f} ms CPU"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Go2Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
