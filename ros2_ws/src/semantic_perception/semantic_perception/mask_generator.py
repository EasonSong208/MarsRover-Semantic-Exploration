"""Utilities for generating and summarizing fake semantic masks."""

from typing import Dict

import numpy as np


CLASS_IDS = np.array([0, 1, 2, 3, 4, 255], dtype=np.uint8)
CLASS_PROBABILITIES = np.array(
    [0.55, 0.25, 0.10, 0.05, 0.02, 0.03],
    dtype=np.float64,
)
RATIO_CLASS_IDS = {
    'hill_ratio': 1,
    'crater_ratio': 2,
    'step_ratio': 3,
    'rover_ratio': 4,
}


def generate_fake_mask(
    height: int,
    width: int,
    rng: np.random.Generator,
) -> np.ndarray:
    """Return a random mono8 semantic mask with the requested dimensions."""
    if height <= 0 or width <= 0:
        raise ValueError('height and width must both be positive')

    return rng.choice(
        CLASS_IDS,
        size=(height, width),
        p=CLASS_PROBABILITIES,
    ).astype(np.uint8, copy=False)


def calculate_class_ratios(mask: np.ndarray) -> Dict[str, float]:
    """Calculate selected class ratios using all mask pixels as denominator."""
    if mask.ndim != 2 or mask.size == 0:
        raise ValueError('mask must be a non-empty two-dimensional array')

    pixel_count = mask.size
    return {
        name: float(np.count_nonzero(mask == class_id) / pixel_count)
        for name, class_id in RATIO_CLASS_IDS.items()
    }
