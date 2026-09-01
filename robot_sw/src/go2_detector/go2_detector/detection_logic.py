"""Pure logic behind the go2 detector node — no ROS, no torch, no ultralytics.

Everything the node can get quietly wrong (the sim-time throttle, the class filter, the
pixel-box conversion, the raw-image decode) lives here so it is unit testable on a plain
machine with no ROS, no model and no GPU:

    cd robot_sw/src/go2_detector && python3 -m pytest test -q
The node file next door holds only the ROS plumbing and the model call.

Imports are numpy + stdlib on purpose. numpy is the ONE dependency because the image
decode is genuinely array work, and it is already present on every ROS 2 Jazzy image
(distro numpy 1.26.4, pinned in ../../constraints-detector.txt).
"""

import numpy as np

#: Image encodings the node understands. The go2 camera this app is developed against
#: publishes `rgb8` (measured: /camera/image_raw, rgb8, 640x480, step 1920); `bgr8` is
#: accepted too because that is what most real camera drivers emit and this app has to
#: run against those as well.
SUPPORTED_ENCODINGS = ("rgb8", "bgr8")

#: Throttle comparison slack, in seconds. Without it the throttle silently halves its
#: own rate on exact-multiple stamps: with 10 Hz input and a 5 Hz target,
#: `0.3 - 0.1 == 0.19999999999999998 < 0.2` in IEEE-754 double, so every second frame
#: after the first would be dropped and the node would run at 3.3 Hz. 1 us is far below
#: the 5 ms sim step (fixed_dt 0.005) so it can never let an extra frame through.
_PERIOD_EPS_S = 1e-6


def stamp_to_seconds(sec, nanosec):
    """ROS time -> float seconds. The node throttles on the IMAGE stamp, not on its own
    clock: the stamp is sim time (every sensor message is stamped with the sim clock), so
    the detector's duty cycle is identical whatever the wall-clock RTF is."""
    return float(sec) + float(nanosec) * 1e-9


def should_process(stamp_s, last_stamp_s, min_period_s):
    """True when a frame stamped `stamp_s` is due, given the last processed stamp.

    `last_stamp_s is None` (first frame) always processes. A stamp that moves BACKWARDS
    also processes and re-syncs: a new sample in a batch run restarts sim time at 0 and
    a detector that waited for `last + period` would go blind for the rest of the run.
    """
    if last_stamp_s is None:
        return True
    if stamp_s < last_stamp_s:
        return True
    if min_period_s <= 0.0:
        return True
    return (stamp_s - last_stamp_s) >= (min_period_s - _PERIOD_EPS_S)


def image_to_bgr(height, width, step, encoding, data):
    """`sensor_msgs/Image` payload -> contiguous HxWx3 BGR uint8 array.

    BGR because that is what ultralytics assumes for numpy sources (the cv2 convention);
    handing it RGB silently costs confidence instead of failing. `step` is honoured, so a
    padded row layout decodes correctly rather than shearing the image diagonally.
    Raises ValueError on anything it cannot decode — the node logs that once and drops
    the frame instead of feeding the model garbage.
    """
    if encoding not in SUPPORTED_ENCODINGS:
        raise ValueError(f"unsupported encoding {encoding!r} (need one of {SUPPORTED_ENCODINGS})")
    row_bytes = width * 3
    if height <= 0 or width <= 0 or step < row_bytes:
        raise ValueError(f"bad image geometry {width}x{height} step={step}")
    buf = np.frombuffer(data, dtype=np.uint8)
    if buf.size < height * step:
        raise ValueError(f"short image buffer: {buf.size} B < {height * step} B")
    frame = buf[: height * step].reshape(height, step)[:, :row_bytes].reshape(height, width, 3)
    if encoding == "rgb8":
        frame = frame[:, :, ::-1]
    # ascontiguousarray: the RGB->BGR flip above is a negative-stride view and OpenCV
    # rejects those inside ultralytics' letterbox.
    return np.ascontiguousarray(frame)


def to_bbox(x1, y1, x2, y2):
    """Corner box (pixels) -> `vision_msgs/BoundingBox2D` fields (cx, cy, size_x, size_y).

    Corners are normalised, so a box given as (x2 < x1) still yields a positive size.
    """
    x_lo, x_hi = (x1, x2) if x1 <= x2 else (x2, x1)
    y_lo, y_hi = (y1, y2) if y1 <= y2 else (y2, y1)
    return (
        (x_lo + x_hi) / 2.0,
        (y_lo + y_hi) / 2.0,
        x_hi - x_lo,
        y_hi - y_lo,
    )


def select_detections(raw, conf_threshold, class_whitelist):
    """Filter + convert model output.

    `raw` = iterable of `(class_name, score, (x1, y1, x2, y2))` in pixels.
    Returns `[(class_name, score, (cx, cy, size_x, size_y)), ...]`.

    The whitelist is EMPTY by default and an empty whitelist means "publish everything"
    (the detector does not know the mission; the class decision belongs to the
    tracker/manager). It exists only so a deployment can cut the topic down without
    touching this node.
    """
    allowed = frozenset(class_whitelist or ())
    kept = []
    for name, score, box in raw:
        if float(score) < conf_threshold:
            continue
        if allowed and name not in allowed:
            continue
        kept.append((name, float(score), to_bbox(*box)))
    return kept
