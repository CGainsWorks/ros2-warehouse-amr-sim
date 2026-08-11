"""Voxel indexing helpers shared by clustering and the 3D occupancy grid.

Kept separate because both the clustering front end and the occupancy publisher
need the exact same discretisation. If they disagree by even one voxel the
occupancy grid stops lining up with the clusters that produced it, which is an
unpleasant bug to chase in RViz.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

# int64 has room for ~9.2e18. Packing three voxel axes into one integer is much
# faster than np.unique(axis=0) and, unlike it, behaves identically across the
# numpy 1.x/2.x boundary where the shape of `return_inverse` changed.
_MAX_PACKED = np.int64(2) ** 62


def voxel_indices(
    points: np.ndarray,
    resolution: float,
    origin: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return integer voxel coordinates and the origin they are measured from.

    ``origin`` defaults to the per-axis minimum of ``points`` so that all
    returned indices are non-negative.
    """
    if resolution <= 0.0:
        raise ValueError("resolution must be > 0, got %r" % (resolution,))
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3), got %r" % (points.shape,))

    if origin is None:
        origin = points.min(axis=0) if len(points) else np.zeros(3)
    origin = np.asarray(origin, dtype=np.float64).reshape(3)

    idx = np.floor((points - origin) / resolution).astype(np.int64)
    return idx, origin


def pack_indices(idx: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Pack non-negative-shifted voxel coordinates into one int64 per point.

    Returns ``(packed, shift, dims)`` where ``shift`` is the per-axis offset
    applied and ``dims`` the per-axis extent, both of which are needed to unpack.
    """
    if len(idx) == 0:
        return np.empty(0, dtype=np.int64), np.zeros(3, dtype=np.int64), np.ones(3, dtype=np.int64)

    shift = idx.min(axis=0)
    shifted = idx - shift
    dims = shifted.max(axis=0) + 1

    # Guard rather than silently aliasing two distant voxels onto one key.
    span = np.prod(dims.astype(object))
    if span > int(_MAX_PACKED):
        raise ValueError(
            "voxel grid spans %d cells, which overflows int64 packing; "
            "increase the voxel resolution or crop the cloud first" % span
        )

    packed = (shifted[:, 2] * dims[1] + shifted[:, 1]) * dims[0] + shifted[:, 0]
    return packed, shift, dims


def unpack_indices(packed: np.ndarray, shift: np.ndarray, dims: np.ndarray) -> np.ndarray:
    """Inverse of :func:`pack_indices`."""
    packed = np.asarray(packed, dtype=np.int64)
    x = packed % dims[0]
    rest = packed // dims[0]
    y = rest % dims[1]
    z = rest // dims[1]
    return np.stack([x, y, z], axis=1) + shift


def voxel_downsample(
    points: np.ndarray,
    resolution: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Reduce ``points`` to one centroid per occupied voxel.

    Returns ``(centroids, inverse, counts)``:

    * ``centroids`` – (V, 3) mean position of the points in each occupied voxel.
    * ``inverse``   – (N,) index into ``centroids`` for every input point, so a
      label computed on the downsampled cloud can be pushed back onto the full
      resolution cloud with a single fancy-index.
    * ``counts``    – (V,) number of original points behind each centroid.

    Centroids rather than voxel centres: a warehouse LiDAR puts very uneven
    numbers of returns in each voxel, and snapping to centres visibly quantises
    the cluster bounding boxes that the zone publisher then reasons about.
    """
    points = np.asarray(points, dtype=np.float64)
    if len(points) == 0:
        return np.empty((0, 3)), np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64)

    idx, _ = voxel_indices(points, resolution)
    packed, _, _ = pack_indices(idx)

    _, inverse, counts = np.unique(packed, return_inverse=True, return_counts=True)
    inverse = np.asarray(inverse).reshape(-1)
    n_vox = len(counts)

    centroids = np.empty((n_vox, 3), dtype=np.float64)
    for axis in range(3):
        centroids[:, axis] = np.bincount(
            inverse, weights=points[:, axis], minlength=n_vox
        ) / counts

    return centroids, inverse, counts
