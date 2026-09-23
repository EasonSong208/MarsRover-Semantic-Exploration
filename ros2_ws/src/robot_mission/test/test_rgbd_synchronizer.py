"""Pure-Python tests for bounded RGB/depth matching and soft health states."""

import math

from robot_mission.rgbd_synchronizer import (
    DIRECT_SLOP, FIXED_OFFSET, ConsecutiveMatchGate, OffsetEstimator,
    RgbDepthSynchronizer, SensorSyncState, classify_sensor_state,
)


def feed_pairs(sync, count=100, depth_delay_ms=55.0):
    matches = []
    for index in range(count):
        rgb_stamp = index / 30.0
        jitter_ms = (-5.0, 0.0, 5.0)[index % 3]
        depth_stamp = rgb_stamp + (depth_delay_ms + jitter_ms) / 1000.0
        new, _ = sync.add_rgb(f'rgb-{index}', rgb_stamp, rgb_stamp)
        matches.extend(new)
        new, _ = sync.add_depth(f'depth-{index}', depth_stamp, depth_stamp)
        matches.extend(new)
    return matches


def test_direct_70ms_slop_matches_fixed_55ms_delay_with_jitter():
    sync = RgbDepthSynchronizer(DIRECT_SLOP, 70.0, queue_size=20)
    matches = feed_pairs(sync)
    assert len(matches) >= 95
    assert sync.summary()['match_rate'] >= 0.95
    assert max(abs(item.raw_delta_ms) for item in matches) <= 60.1


def test_fixed_negative_55ms_offset_reduces_corrected_delta():
    sync = RgbDepthSynchronizer(
        FIXED_OFFSET, 25.0, depth_stamp_offset_ms=-55.0,
        queue_size=20)
    matches = feed_pairs(sync)
    assert len(matches) >= 95
    mean_raw = sum(abs(item.raw_delta_ms) for item in matches) / len(matches)
    mean_corrected = (
        sum(abs(item.corrected_delta_ms) for item in matches) / len(matches))
    assert mean_raw > 50.0
    assert mean_corrected < 4.0


def test_missing_depth_frames_recover_without_recreating_synchronizer():
    sync = RgbDepthSynchronizer(DIRECT_SLOP, 70.0, queue_size=20)
    matches = []
    for index in range(20):
        stamp = index * 0.1
        new, _ = sync.add_rgb(index, stamp, stamp)
        matches.extend(new)
        if index not in {5, 6, 7}:
            new, _ = sync.add_depth(index, stamp + 0.02, stamp + 0.02)
            matches.extend(new)
    assert any(match.rgb.message == 4 for match in matches)
    assert any(match.rgb.message >= 8 for match in matches)
    assert sync.matched_count >= 16


def test_two_second_depth_stop_is_recoverable_depth_stale():
    state = classify_sensor_state(
        3.0, rgb_arrival_time=3.0, depth_arrival_time=1.0,
        matched_depth_arrival_time=1.0,
        depth_freshness_limit_sec=0.15,
        sensor_stop_timeout_sec=0.5,
        sensor_warn_timeout_sec=1.0)
    assert state is SensorSyncState.DEPTH_STALE
    recovered = classify_sensor_state(
        3.1, rgb_arrival_time=3.1, depth_arrival_time=3.1,
        matched_depth_arrival_time=3.1,
        depth_freshness_limit_sec=0.15,
        sensor_stop_timeout_sec=0.5,
        sensor_warn_timeout_sec=1.0)
    assert recovered is SensorSyncState.SYNC_OK


def test_out_of_order_arrival_uses_closest_message_once():
    sync = RgbDepthSynchronizer(DIRECT_SLOP, 70.0, queue_size=10)
    sync.add_depth('depth-far', 1.060, 1.0)
    sync.add_depth('depth-close', 1.012, 1.1)
    matches, _ = sync.add_rgb('rgb', 1.010, 1.2)
    assert len(matches) == 1
    assert matches[0].depth.message == 'depth-close'
    assert all(item.message != 'depth-close' for item in sync.depth_buffer)
    matches, _ = sync.add_rgb('rgb-2', 1.058, 1.3)
    assert matches[0].depth.message == 'depth-far'


def test_unmatchable_streams_remain_bounded_and_drop_old_messages():
    sync = RgbDepthSynchronizer(
        FIXED_OFFSET, 5.0, queue_size=4, max_buffer_age_ms=200.0)
    for index in range(50):
        sync.add_rgb(index, index * 0.1, index * 0.1)
    assert len(sync.rgb_buffer) <= 4
    assert sync.dropped_old_rgb_count >= 46
    assert sync.matched_count == 0


def test_offset_estimator_sign_is_negative_for_late_depth():
    estimator = OffsetEstimator(20)
    for index in range(20):
        stamp = index * 0.1
        estimator.add_rgb(stamp)
        estimator.add_depth(stamp + 0.055)
    assert math.isclose(estimator.estimated_depth_stamp_offset_ms, -55.0)


def test_recent_raw_depth_without_match_is_rgb_only():
    state = classify_sensor_state(
        10.0, rgb_arrival_time=10.0, depth_arrival_time=10.0,
        matched_depth_arrival_time=9.8,
        depth_freshness_limit_sec=0.15,
        sensor_stop_timeout_sec=0.5,
        sensor_warn_timeout_sec=1.0)
    assert state is SensorSyncState.RGB_ONLY


def test_unmatched_depth_beyond_stop_timeout_forces_depth_stale():
    state = classify_sensor_state(
        10.0, rgb_arrival_time=10.0, depth_arrival_time=10.0,
        matched_depth_arrival_time=9.0,
        depth_freshness_limit_sec=0.15,
        sensor_stop_timeout_sec=0.5,
        sensor_warn_timeout_sec=1.0)
    assert state is SensorSyncState.DEPTH_STALE


def test_recovery_requires_two_consecutive_matches_and_can_reset():
    gate = ConsecutiveMatchGate(required_count=2)
    gate.mark_match()
    assert not gate.ready
    gate.mark_match()
    assert gate.ready
    gate.mark_unhealthy()
    assert not gate.ready and gate.count == 0
