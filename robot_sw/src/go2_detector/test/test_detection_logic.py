"""Unit tests for the detector's pure logic (no ROS, no model, no GPU).

    cd robot_sw/src/go2_detector && python3 -m pytest test -q
"""

import numpy as np
import pytest

from go2_detector.detection_logic import (
    image_to_bgr,
    select_detections,
    should_process,
    stamp_to_seconds,
    to_bbox,
)

PERIOD_5HZ = 1.0 / 5.0


def test_stamp_to_seconds_combines_sec_and_nanosec():
    assert stamp_to_seconds(12, 500_000_000) == pytest.approx(12.5)


def test_first_frame_always_processes():
    assert should_process(0.0, None, PERIOD_5HZ) is True


def test_frame_inside_the_period_is_dropped():
    assert should_process(0.1, 0.0, PERIOD_5HZ) is False


def test_throttle_keeps_the_target_rate_on_float_edges():
    """10 Hz in, 5 Hz target -> exactly every other frame.

    This is the regression test for the float trap: `0.3 - 0.1` is
    0.19999999999999998, so a naive `>= 0.2` drops the frame and the node quietly runs
    at 3.3 Hz instead of 5 Hz.
    """
    last = None
    processed = []
    for i in range(11):
        stamp = i * 0.1
        if should_process(stamp, last, PERIOD_5HZ):
            processed.append(round(stamp, 6))
            last = stamp
    assert processed == [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]


def test_clock_rewind_resyncs_instead_of_going_blind():
    """A new batch sample restarts sim time at 0; waiting for `last + period` would
    blind the detector for the whole run."""
    assert should_process(0.01, 57.4, PERIOD_5HZ) is True


def test_zero_rate_means_process_every_frame():
    assert should_process(0.001, 0.0, 0.0) is True


def test_to_bbox_returns_center_and_size():
    assert to_bbox(10.0, 20.0, 30.0, 60.0) == (20.0, 40.0, 20.0, 40.0)


def test_to_bbox_normalises_reversed_corners():
    assert to_bbox(30.0, 60.0, 10.0, 20.0) == (20.0, 40.0, 20.0, 40.0)


def _raw():
    return [
        ("person", 0.81, (292.6, 18.7, 347.4, 255.4)),
        ("chair", 0.36, (0.3, 216.0, 120.6, 293.5)),
        ("bench", 0.57, (6.1, 153.0, 137.6, 232.6)),
    ]


def test_empty_whitelist_publishes_every_class():
    kept = select_detections(_raw(), 0.25, [])
    assert [name for name, _, _ in kept] == ["person", "chair", "bench"]


def test_whitelist_filters_by_class_name():
    kept = select_detections(_raw(), 0.25, ["person", "chair"])
    assert [name for name, _, _ in kept] == ["person", "chair"]


def test_conf_threshold_drops_weak_detections():
    kept = select_detections(_raw(), 0.5, [])
    assert [name for name, _, _ in kept] == ["person", "bench"]


def test_select_detections_converts_boxes():
    kept = select_detections([("person", 0.9, (10, 20, 30, 60))], 0.25, [])
    assert kept == [("person", 0.9, (20.0, 40.0, 20.0, 40.0))]


def _rgb_frame(height, width, pad=0):
    """HxWx3 RGB image with a distinct value per channel, plus optional row padding."""
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[:, :, 0] = 10  # R
    rgb[:, :, 1] = 20  # G
    rgb[:, :, 2] = 30  # B
    step = width * 3 + pad
    buf = np.zeros((height, step), dtype=np.uint8)
    buf[:, : width * 3] = rgb.reshape(height, width * 3)
    return buf.tobytes(), step


def test_rgb8_is_converted_to_bgr():
    data, step = _rgb_frame(4, 5)
    out = image_to_bgr(4, 5, step, "rgb8", data)
    assert out.shape == (4, 5, 3)
    assert tuple(out[0, 0]) == (30, 20, 10)  # B, G, R
    assert out.flags["C_CONTIGUOUS"]  # ultralytics/cv2 reject negative strides


def test_bgr8_is_passed_through():
    data, step = _rgb_frame(4, 5)
    out = image_to_bgr(4, 5, step, "bgr8", data)
    assert tuple(out[0, 0]) == (10, 20, 30)


def test_row_padding_does_not_shear_the_image():
    data, step = _rgb_frame(3, 4, pad=7)
    out = image_to_bgr(3, 4, step, "bgr8", data)
    assert out.shape == (3, 4, 3)
    assert (out[:, :, 0] == 10).all() and (out[:, :, 2] == 30).all()


def test_unsupported_encoding_raises():
    with pytest.raises(ValueError, match="unsupported encoding"):
        image_to_bgr(4, 5, 15, "16UC1", b"\x00" * 60)


def test_short_buffer_raises():
    with pytest.raises(ValueError, match="short image buffer"):
        image_to_bgr(4, 5, 15, "rgb8", b"\x00" * 30)


def test_bad_geometry_raises():
    with pytest.raises(ValueError, match="bad image geometry"):
        image_to_bgr(4, 5, 4, "rgb8", b"\x00" * 60)
