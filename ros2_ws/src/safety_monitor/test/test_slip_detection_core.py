import math

from safety_monitor.slip_detection_core import (
    DetectorConfig,
    DetectorState,
    SlipDetectorCore,
    normalize_angle,
)


def feed_orientation(detector, start, end, step=0.05):
    t = start
    while t <= end + 1.0e-9:
        detector.set_orientation(t, 0.0, 0.0)
        t += step


def feed_rotation(detector, start, end, expected_rate, observed_rate, step=0.05):
    t = start
    while t <= end + 1.0e-9:
        detector.add_odom(t, 0.0, 0.0, expected_rate * t)
        detector.add_gyro(t, observed_rate)
        detector.set_orientation(t, 0.0, 0.0)
        detector.set_command(t, 0.0, 0.0, expected_rate)
        t += step


def test_normalize_angle_wraps_both_directions():
    assert math.isclose(normalize_angle(3.0 * math.pi), -math.pi)
    assert math.isclose(normalize_angle(-1.5 * math.pi), 0.5 * math.pi)


def test_matching_rotation_arms_without_trip():
    config = DetectorConfig(rotation_confirmations=2, slam_resume_stable_time=0.0)
    detector = SlipDetectorCore(config)
    feed_rotation(detector, 0.0, 1.0, expected_rate=1.0, observed_rate=1.0)
    result = detector.evaluate(1.0)
    assert result.state == DetectorState.ARMED
    assert abs(result.yaw_error) < 1.0e-6


def test_rotation_mismatch_trips_after_confirmations():
    config = DetectorConfig(rotation_confirmations=3)
    detector = SlipDetectorCore(config)
    feed_rotation(detector, 0.0, 1.0, expected_rate=1.0, observed_rate=0.1)
    assert detector.evaluate(1.0).state == DetectorState.SUSPECT
    assert detector.evaluate(1.0).state == DetectorState.SUSPECT
    result = detector.evaluate(1.0)
    assert result.state == DetectorState.TRIPPED
    assert result.reason == 'rotation_mismatch'
    assert result.stop_requested


def test_trip_is_latched_until_stationary_manual_reset():
    config = DetectorConfig(rotation_confirmations=1)
    detector = SlipDetectorCore(config)
    feed_rotation(detector, 0.0, 1.0, expected_rate=1.0, observed_rate=0.0)
    assert detector.evaluate(1.0).state == DetectorState.TRIPPED
    success, _ = detector.reset(1.0)
    assert not success
    detector.set_command(1.05, 0.0, 0.0, 0.0)
    detector.add_gyro(1.05, 0.0)
    detector.set_orientation(1.05, 0.0, 0.0)
    success, reason = detector.reset(1.05)
    assert success, reason


def test_rf2o_detects_longitudinal_execution_shortfall():
    config = DetectorConfig(
        linear_confirmations=2,
        rotation_min_expected=10.0,
        linear_abs_error=0.03,
        linear_rel_error=0.30,
    )
    detector = SlipDetectorCore(config)
    t = 0.0
    while t <= 1.2 + 1.0e-9:
        detector.add_odom(t, 0.20 * t, 0.0, 0.0)
        detector.add_rf2o(t, 0.03 * t, 0.0, 0.0)
        detector.add_gyro(t, 0.0)
        detector.set_orientation(t, 0.0, 0.0)
        detector.set_command(t, 0.20, 0.0, 0.0)
        t += 0.05
    assert detector.evaluate(1.20).state == DetectorState.SUSPECT
    result = detector.evaluate(1.20)
    assert result.state == DetectorState.TRIPPED
    assert result.reason == 'linear_mismatch'


def test_tilt_blocks_slam_and_excessive_tilt_trips():
    detector = SlipDetectorCore(DetectorConfig(slam_resume_stable_time=0.0))
    feed_rotation(detector, 0.0, 0.8, expected_rate=0.0, observed_rate=0.0)
    detector.evaluate(0.8)
    detector.set_orientation(0.81, math.radians(10.0), 0.0)
    tilted = detector.evaluate(0.81)
    assert tilted.tilted
    assert not tilted.slam_scan_allowed
    detector.set_orientation(0.82, math.radians(16.0), 0.0)
    result = detector.evaluate(0.82)
    assert result.state == DetectorState.TRIPPED
    assert result.reason == 'excessive_tilt'
