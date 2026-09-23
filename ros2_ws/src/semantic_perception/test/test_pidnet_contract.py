"""Tests for the pure hazard5 PIDNet inference contract."""

import numpy as np
import pytest

from semantic_perception.pidnet_contract import (
    CLASS_COLORS_RGB,
    CLASS_NAMES,
    calculate_ratios,
    colorize_prediction,
    make_overlay,
    validate_prediction,
)


def test_hazard5_class_order_has_no_ignore_output():
    """Inference classes must exactly match the accepted five-class order."""
    assert CLASS_NAMES == (
        'other',
        'hill_candidate',
        'crater_candidate',
        'step_candidate',
        'rover',
    )


def test_ratios_and_palette_follow_class_ids():
    """Class IDs must map deterministically to ratios and RGB colors."""
    mask = np.array([[0, 1, 2, 3, 4], [0, 0, 4, 3, 2]], dtype=np.uint8)
    assert calculate_ratios(mask) == {
        'other_ratio': 0.3,
        'hill_candidate_ratio': 0.1,
        'crater_candidate_ratio': 0.2,
        'step_candidate_ratio': 0.2,
        'rover_ratio': 0.2,
    }
    color = colorize_prediction(mask)
    assert color.shape == (2, 5, 3)
    assert np.array_equal(color[0], CLASS_COLORS_RGB)


def test_overlay_preserves_shape_and_dtype():
    """Debug overlay must remain a publishable uint8 RGB image."""
    rgb = np.full((2, 3, 3), 100, dtype=np.uint8)
    mask = np.array([[0, 1, 2], [3, 4, 0]], dtype=np.uint8)
    overlay = make_overlay(rgb, mask)
    assert overlay.shape == rgb.shape
    assert overlay.dtype == np.uint8


@pytest.mark.parametrize(
    'mask',
    [
        np.array([[255]], dtype=np.uint8),
        np.array([[5]], dtype=np.uint8),
        np.array([[1]], dtype=np.int64),
        np.array([], dtype=np.uint8),
    ],
)
def test_prediction_rejects_training_ignore_and_invalid_arrays(mask):
    """The deployed inference output must never contain 255 or invalid IDs."""
    with pytest.raises(ValueError):
        validate_prediction(mask)
