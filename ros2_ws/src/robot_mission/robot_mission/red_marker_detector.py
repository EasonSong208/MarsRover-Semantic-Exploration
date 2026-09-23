"""ROS-independent red-marker detection and reference statistics."""

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorConfig:
    """Tunable HSV, morphology, area, and depth gates."""

    hsv_low_1: Tuple[int, int, int] = (0, 100, 60)
    hsv_high_1: Tuple[int, int, int] = (12, 255, 255)
    hsv_low_2: Tuple[int, int, int] = (168, 100, 60)
    hsv_high_2: Tuple[int, int, int] = (179, 255, 255)
    morphology_kernel_size: int = 5
    morphology_iterations: int = 1
    depth_erode_kernel_size: int = 7
    min_marker_area: float = 500.0
    max_marker_area: float = 120000.0
    min_valid_depth_pixels: int = 50


@dataclass(frozen=True)
class MarkerDetection:
    """One marker observation. Depth retains the source image's native unit."""

    detected: bool = False
    valid: bool = False
    reason: str = 'marker_not_found'
    u: float = 0.0
    v: float = 0.0
    depth: float = 0.0
    area: float = 0.0
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)
    valid_depth_count: int = 0
    mask: Optional[np.ndarray] = None


@dataclass(frozen=True)
class ReferenceStats:
    """Robust multi-frame reference and derived completion tolerances."""

    sample_count: int
    u: float
    v: float
    depth: float
    bbox_width: float
    area: float
    u_mad: float
    depth_mad: float
    yaw_tolerance_px: float
    depth_tolerance: float


def _validate_config(config: DetectorConfig) -> None:
    if config.morphology_kernel_size < 1:
        raise ValueError('morphology_kernel_size must be positive')
    if config.depth_erode_kernel_size < 1:
        raise ValueError('depth_erode_kernel_size must be positive')
    if config.morphology_iterations < 0:
        raise ValueError('morphology_iterations cannot be negative')
    if not 0 < config.min_marker_area < config.max_marker_area:
        raise ValueError('marker area limits are invalid')
    if config.min_valid_depth_pixels < 1:
        raise ValueError('min_valid_depth_pixels must be positive')


def _kernel(size: int) -> np.ndarray:
    size = size if size % 2 else size + 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))


def detect_red_marker(
    rgb_image: np.ndarray,
    depth_image: np.ndarray,
    config: DetectorConfig = DetectorConfig(),
) -> MarkerDetection:
    """
    Detect the largest red component in an RGB8 image.

    The depth median is computed inside an eroded component mask. Zero and
    non-finite samples are discarded; no conversion or unit assumption is made.
    """
    _validate_config(config)
    if (rgb_image.ndim != 3 or rgb_image.shape[2] != 3
            or rgb_image.dtype != np.uint8):
        raise ValueError('rgb_image must be an RGB8 HxWx3 array')
    if depth_image.ndim != 2 or depth_image.shape != rgb_image.shape[:2]:
        raise ValueError('depth_image must match RGB height and width')

    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    mask_1 = cv2.inRange(
        hsv, np.asarray(config.hsv_low_1), np.asarray(config.hsv_high_1))
    mask_2 = cv2.inRange(
        hsv, np.asarray(config.hsv_low_2), np.asarray(config.hsv_high_2))
    mask = cv2.bitwise_or(mask_1, mask_2)
    morph_kernel = _kernel(config.morphology_kernel_size)
    if config.morphology_iterations:
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, morph_kernel,
            iterations=config.morphology_iterations)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, morph_kernel,
            iterations=config.morphology_iterations)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return MarkerDetection(mask=mask)
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    moments = cv2.moments(contour)
    bbox = tuple(int(value) for value in cv2.boundingRect(contour))
    if moments['m00'] <= 0.0:
        return MarkerDetection(
            detected=True, reason='invalid_contour', area=area, bbox=bbox,
            mask=mask)
    u = float(moments['m10'] / moments['m00'])
    v = float(moments['m01'] / moments['m00'])
    if area < config.min_marker_area or area > config.max_marker_area:
        return MarkerDetection(
            detected=True, reason='marker_area_out_of_range', u=u, v=v,
            area=area, bbox=bbox, mask=mask)

    component_mask = np.zeros_like(mask)
    cv2.drawContours(component_mask, [contour], -1, 255, thickness=-1)
    depth_mask = cv2.erode(
        component_mask, _kernel(config.depth_erode_kernel_size), iterations=1)
    samples = depth_image[depth_mask != 0]
    valid_samples = samples[np.isfinite(samples) & (samples > 0)]
    valid_count = int(valid_samples.size)
    if valid_count < config.min_valid_depth_pixels:
        return MarkerDetection(
            detected=True, reason='insufficient_valid_depth', u=u, v=v,
            area=area, bbox=bbox, valid_depth_count=valid_count, mask=mask)
    return MarkerDetection(
        detected=True, valid=True, reason='ok', u=u, v=v,
        depth=float(np.median(valid_samples)), area=area, bbox=bbox,
        valid_depth_count=valid_count, mask=mask)


def median_absolute_deviation(values: Iterable[float]) -> float:
    """Return the unscaled median absolute deviation."""
    array = np.asarray(tuple(values), dtype=np.float64)
    if array.size == 0:
        raise ValueError('at least one sample is required')
    median = np.median(array)
    return float(np.median(np.abs(array - median)))


def build_reference(
    detections: Iterable[MarkerDetection],
    configured_yaw_tolerance_px: float,
    depth_relative_tolerance: float,
) -> ReferenceStats:
    """Build median/MAD reference values from valid observations."""
    samples = tuple(detection for detection in detections if detection.valid)
    if not samples:
        raise ValueError('at least one valid detection is required')
    if configured_yaw_tolerance_px <= 0.0:
        raise ValueError('configured_yaw_tolerance_px must be positive')
    if not 0.0 < depth_relative_tolerance < 1.0:
        raise ValueError('depth_relative_tolerance must be between zero and one')
    u_values = np.asarray([item.u for item in samples], dtype=np.float64)
    v_values = np.asarray([item.v for item in samples], dtype=np.float64)
    depth_values = np.asarray(
        [item.depth for item in samples], dtype=np.float64)
    widths = np.asarray([item.bbox[2] for item in samples], dtype=np.float64)
    areas = np.asarray([item.area for item in samples], dtype=np.float64)
    u = float(np.median(u_values))
    depth = float(np.median(depth_values))
    u_mad = median_absolute_deviation(u_values)
    depth_mad = median_absolute_deviation(depth_values)
    return ReferenceStats(
        sample_count=len(samples), u=u, v=float(np.median(v_values)),
        depth=depth, bbox_width=float(np.median(widths)),
        area=float(np.median(areas)), u_mad=u_mad,
        depth_mad=depth_mad,
        yaw_tolerance_px=max(configured_yaw_tolerance_px, 3.0 * u_mad),
        depth_tolerance=max(
            3.0 * depth_mad, depth * depth_relative_tolerance),
    )


def depth_preview(depth_image: np.ndarray) -> np.ndarray:
    """Create a visual-only color preview without modifying raw depth data."""
    valid = np.isfinite(depth_image) & (depth_image > 0)
    scaled = np.zeros(depth_image.shape, dtype=np.uint8)
    if np.any(valid):
        low, high = np.percentile(depth_image[valid], (2.0, 98.0))
        if high <= low:
            high = low + 1.0
        scaled[valid] = np.clip(
            (depth_image[valid].astype(np.float64) - low) * 255.0
            / (high - low), 0, 255).astype(np.uint8)
    return cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
