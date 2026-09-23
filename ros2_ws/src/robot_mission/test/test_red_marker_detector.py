"""Synthetic-image tests for red marker detection and reference statistics."""

import pytest

np = pytest.importorskip('numpy')
cv2 = pytest.importorskip('cv2')

from robot_mission.red_marker_detector import (  # noqa: E402
    DetectorConfig, MarkerDetection, build_reference, detect_red_marker,
)


CONFIG = DetectorConfig(
    morphology_kernel_size=3, depth_erode_kernel_size=3,
    min_marker_area=20.0, max_marker_area=10000.0,
    min_valid_depth_pixels=4,
)


def blank_pair(width=100, height=80, depth_value=1000):
    return (
        np.zeros((height, width, 3), dtype=np.uint8),
        np.full((height, width), depth_value, dtype=np.uint16),
    )


def test_dual_hsv_red_intervals_detect_low_and_high_hue():
    depth = np.full((60, 100), 900, dtype=np.uint16)
    hsv = np.zeros((60, 100, 3), dtype=np.uint8)
    hsv[10:30, 10:30] = (2, 255, 255)
    hsv[10:35, 55:85] = (175, 255, 255)
    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    detection = detect_red_marker(rgb, depth, CONFIG)
    assert detection.valid
    assert detection.u > 55  # Larger high-hue component was also recognized.


def test_largest_red_region_is_selected():
    rgb, depth = blank_pair()
    rgb[5:20, 5:20] = (255, 0, 0)
    rgb[30:70, 50:95] = (255, 0, 0)
    detection = detect_red_marker(rgb, depth, CONFIG)
    assert detection.valid
    assert detection.bbox[0] >= 49
    assert detection.area > 1000


def test_rgb8_channel_order_does_not_mistake_blue_for_red():
    rgb, depth = blank_pair()
    rgb[10:50, 10:50] = (0, 0, 255)  # Blue in RGB ordering.
    assert not detect_red_marker(rgb, depth, CONFIG).detected
    rgb[10:50, 10:50] = (255, 0, 0)
    assert detect_red_marker(rgb, depth, CONFIG).valid


def test_eroded_interior_depth_uses_median_not_contour_edge():
    rgb, depth = blank_pair(60, 60, 0)
    rgb[10:50, 10:50] = (255, 0, 0)
    depth[10:50, 10:50] = 4000
    depth[15:45, 15:45] = 777
    config = DetectorConfig(
        morphology_kernel_size=3, depth_erode_kernel_size=11,
        min_marker_area=20, max_marker_area=10000,
        min_valid_depth_pixels=10)
    assert detect_red_marker(rgb, depth, config).depth == 777


def test_zero_depth_samples_are_filtered():
    rgb, depth = blank_pair(60, 60, 0)
    rgb[10:50, 10:50] = (255, 0, 0)
    depth[20:40, 20:40] = 1234
    detection = detect_red_marker(rgb, depth, CONFIG)
    assert detection.valid
    assert detection.depth == 1234
    assert detection.valid_depth_count == 400


def sample(u, depth, width=20, area=300):
    return MarkerDetection(
        detected=True, valid=True, reason='ok', u=u, v=40,
        depth=depth, bbox=(0, 0, width, 20), area=area,
        valid_depth_count=100)


def test_reference_uses_multiframe_medians():
    reference = build_reference(
        [sample(10, 100), sample(12, 110), sample(100, 900)], 8, 0.03)
    assert reference.sample_count == 3
    assert reference.u == 12
    assert reference.depth == 110


def test_mad_tolerances_use_three_mad_or_configured_floor():
    reference = build_reference(
        [sample(10, 900), sample(12, 1000), sample(14, 1100)], 4, 0.02)
    assert reference.u_mad == 2
    assert reference.yaw_tolerance_px == 6
    assert reference.depth_mad == 100
    assert reference.depth_tolerance == 300


def test_insufficient_valid_depth_is_explicitly_invalid():
    rgb, depth = blank_pair(60, 60, 0)
    rgb[10:50, 10:50] = (255, 0, 0)
    depth[30, 30] = 1000
    detection = detect_red_marker(rgb, depth, CONFIG)
    assert detection.detected and not detection.valid
    assert detection.reason == 'insufficient_valid_depth'
