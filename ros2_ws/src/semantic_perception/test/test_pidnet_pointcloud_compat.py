"""Tests for the minimal semantic-backend PointCloud2 compatibility helper."""

import numpy as np
import pytest

from semantic_perception.pidnet_pointcloud_compat import coerce_xyz


def test_coerce_xyz_accepts_plain_humble_array():
    """Humble may return a normal Nx3 array for uniform XYZ fields."""
    points = np.array([[1.0, 2.0, 3.0], [4.0, np.nan, 6.0]], dtype=np.float32)
    result = coerce_xyz(points)
    assert result.dtype == np.float64
    assert np.array_equal(result, np.array([[1.0, 2.0, 3.0]]))


def test_coerce_xyz_accepts_structured_array():
    """Other sensor_msgs_py versions may return named structured records."""
    points = np.array(
        [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)],
        dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')],
    )
    assert np.array_equal(
        coerce_xyz(points),
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
    )


def test_coerce_xyz_rejects_incompatible_shapes():
    """The adapter must fail closed for non-XYZ point arrays."""
    with pytest.raises(ValueError):
        coerce_xyz(np.ones((3, 2), dtype=np.float32))
