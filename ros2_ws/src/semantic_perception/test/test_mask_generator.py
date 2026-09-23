"""Tests for fake semantic mask generation."""

import numpy as np
import pytest

from semantic_perception.mask_generator import (
    CLASS_IDS,
    calculate_class_ratios,
    generate_fake_mask,
)


def test_generate_fake_mask_shape_type_and_classes():
    """The generated mask should match the image shape and class contract."""
    mask = generate_fake_mask(24, 32, np.random.default_rng(7))

    assert mask.shape == (24, 32)
    assert mask.dtype == np.uint8
    assert set(np.unique(mask)).issubset(set(CLASS_IDS))


def test_calculate_class_ratios_uses_all_pixels():
    """Ratios should include other and ignore pixels in the denominator."""
    mask = np.array(
        [
            [0, 1, 1, 2],
            [3, 4, 255, 0],
        ],
        dtype=np.uint8,
    )

    assert calculate_class_ratios(mask) == {
        'hill_ratio': 0.25,
        'crater_ratio': 0.125,
        'step_ratio': 0.125,
        'rover_ratio': 0.125,
    }


@pytest.mark.parametrize('height,width', [(0, 1), (1, 0), (-1, 1)])
def test_generate_fake_mask_rejects_invalid_dimensions(height, width):
    """Empty or negative image dimensions should be rejected."""
    with pytest.raises(ValueError):
        generate_fake_mask(height, width, np.random.default_rng(7))
