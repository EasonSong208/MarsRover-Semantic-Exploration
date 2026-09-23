"""Pure NumPy helpers for the PIDNet hazard5 inference contract."""

from typing import Dict

import numpy as np


CLASS_NAMES = (
    'other',
    'hill_candidate',
    'crater_candidate',
    'step_candidate',
    'rover',
)
NUM_CLASSES = len(CLASS_NAMES)
CLASS_COLORS_RGB = np.array(
    [
        [0, 0, 0],
        [230, 159, 0],
        [0, 114, 178],
        [204, 121, 167],
        [0, 158, 115],
    ],
    dtype=np.uint8,
)


def validate_prediction(mask: np.ndarray) -> np.ndarray:
    """Validate and return a contiguous 2-D hazard5 uint8 prediction."""
    if mask.ndim != 2 or mask.size == 0:
        raise ValueError('prediction must be a non-empty two-dimensional array')
    if mask.dtype != np.uint8:
        raise ValueError('prediction must have dtype uint8')
    values = np.unique(mask)
    if np.any(values >= NUM_CLASSES):
        raise ValueError(f'prediction contains unsupported values: {values.tolist()}')
    return np.ascontiguousarray(mask)


def calculate_ratios(mask: np.ndarray) -> Dict[str, float]:
    """Calculate one pixel ratio for every inference class."""
    checked = validate_prediction(mask)
    total = checked.size
    return {
        f'{name}_ratio': float(np.count_nonzero(checked == class_id) / total)
        for class_id, name in enumerate(CLASS_NAMES)
    }


def colorize_prediction(mask: np.ndarray) -> np.ndarray:
    """Convert a class-ID mask to the fixed RGB hazard5 palette."""
    checked = validate_prediction(mask)
    return np.ascontiguousarray(CLASS_COLORS_RGB[checked])


def make_overlay(
    rgb: np.ndarray,
    mask: np.ndarray,
    raw_weight: float = 0.55,
) -> np.ndarray:
    """Blend an RGB image with its fixed-palette semantic prediction."""
    checked = validate_prediction(mask)
    if rgb.dtype != np.uint8 or rgb.shape != checked.shape + (3,):
        raise ValueError('rgb must be uint8 with shape (mask_height, mask_width, 3)')
    if not 0.0 <= raw_weight <= 1.0:
        raise ValueError('raw_weight must be between zero and one')
    colors = colorize_prediction(checked)
    overlay = (
        raw_weight * rgb.astype(np.float32)
        + (1.0 - raw_weight) * colors.astype(np.float32)
    )
    return overlay.round().clip(0, 255).astype(np.uint8)
