"""Small array adapter for ROS2 PointCloud2 NumPy return variants."""

import numpy as np


def coerce_xyz(points: np.ndarray) -> np.ndarray:
    """Return finite Nx3 float64 XYZ from structured or plain arrays."""
    array = np.asarray(points)
    if array.dtype.names:
        required = {'x', 'y', 'z'}
        if not required.issubset(array.dtype.names):
            raise ValueError(
                f'structured point array lacks XYZ fields: {array.dtype.names}')
        xyz = np.column_stack((array['x'], array['y'], array['z']))
    else:
        if array.ndim == 1:
            if array.size % 3 != 0:
                raise ValueError('flat point array size is not divisible by three')
            array = array.reshape((-1, 3))
        if array.ndim != 2 or array.shape[1] < 3:
            raise ValueError(f'plain point array must have shape Nx3, got {array.shape}')
        xyz = array[:, :3]
    xyz = np.asarray(xyz, dtype=np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]
