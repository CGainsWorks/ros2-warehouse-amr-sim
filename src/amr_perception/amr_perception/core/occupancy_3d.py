"""3D voxel occupancy grid over the non-ground returns.

Deliberately not a full probabilistic octree. An OctoMap-style log-odds map is
the right structure when you are accumulating evidence over a long traverse and
need to distinguish "known free" from "unknown". This publisher answers a much
narrower question — which voxels have current returns in them — at 10 Hz for the
zone logic and RViz, and a dense boolean grid over a bounded region does that in
a fraction of the time with none of the tuning.

What it does keep from the probabilistic version is a hit threshold: a voxel
needs ``min_points`` returns before it counts as occupied, which suppresses the
isolated single-return artefacts that a simulated LiDAR produces at grazing
incidence on the floor edge.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .voxel import voxel_indices


@dataclass
class VoxelOccupancy:
    """A dense boolean occupancy grid."""

    origin: np.ndarray  # world position of voxel (0, 0, 0)'s minimum corner
    resolution: float
    grid: np.ndarray  # (nx, ny, nz) bool

    @property
    def dims(self) -> Tuple[int, int, int]:
        return tuple(int(v) for v in self.grid.shape)

    @property
    def occupied_count(self) -> int:
        return int(self.grid.sum())

    def occupied_centres(self) -> np.ndarray:
        """(K, 3) world coordinates of the centre of every occupied voxel."""
        idx = np.argwhere(self.grid)
        if len(idx) == 0:
            return np.empty((0, 3))
        return self.origin + (idx + 0.5) * self.resolution

    def project_2d(self, z_min: Optional[float] = None, z_max: Optional[float] = None) -> np.ndarray:
        """Collapse to an (nx, ny) boolean footprint over a height band.

        This is what the zone logic and the Nav2 obstacle layer actually consume:
        the 3D grid is for visualisation and for the height band selection, but
        the robot plans in the plane.
        """
        k0 = 0 if z_min is None else max(0, int(np.floor((z_min - self.origin[2]) / self.resolution)))
        k1 = (
            self.grid.shape[2]
            if z_max is None
            else min(self.grid.shape[2], int(np.ceil((z_max - self.origin[2]) / self.resolution)))
        )
        if k1 <= k0:
            return np.zeros(self.grid.shape[:2], dtype=bool)
        return self.grid[:, :, k0:k1].any(axis=2)

    def as_dict(self) -> dict:
        return {
            "origin": [float(v) for v in self.origin],
            "resolution": float(self.resolution),
            "dims": list(self.dims),
            "occupied_count": self.occupied_count,
        }


def build_occupancy(
    points: np.ndarray,
    resolution: float = 0.10,
    bounds: Optional[Tuple[np.ndarray, np.ndarray]] = None,
    min_points: int = 2,
) -> VoxelOccupancy:
    """Voxelise ``points`` into an occupancy grid.

    Parameters
    ----------
    resolution
        0.10 m. Half the Nav2 global costmap resolution (0.05 m) would double
        the memory for no planning benefit, since the local costmap that
        consumes this is itself at 0.05 m and the inflation radius is 0.55 m.
        0.10 m keeps a 40 x 25 x 3 m warehouse grid at about 3 MB as bool.
    bounds
        ``(min_xyz, max_xyz)``. Points outside are dropped. Passing explicit
        bounds keeps the grid a fixed size frame to frame, which matters because
        a grid that resizes with the cloud produces an origin that jitters in
        RViz. Defaults to the cloud's own extent.
    min_points
        Returns needed in a voxel before it is marked occupied.
    """
    if resolution <= 0:
        raise ValueError("resolution must be > 0")
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3), got %r" % (points.shape,))

    if bounds is not None:
        lo = np.asarray(bounds[0], dtype=np.float64).reshape(3)
        hi = np.asarray(bounds[1], dtype=np.float64).reshape(3)
        if np.any(hi <= lo):
            raise ValueError("bounds max must exceed bounds min on every axis")
        if len(points):
            keep = np.all((points >= lo) & (points < hi), axis=1)
            points = points[keep]
    elif len(points):
        lo = points.min(axis=0)
        hi = points.max(axis=0) + resolution
    else:
        lo = np.zeros(3)
        hi = np.full(3, resolution)

    dims = np.maximum(1, np.ceil((hi - lo) / resolution).astype(np.int64))
    grid = np.zeros(tuple(int(d) for d in dims), dtype=bool)

    if len(points) == 0:
        return VoxelOccupancy(origin=lo, resolution=resolution, grid=grid)

    idx, _ = voxel_indices(points, resolution, origin=lo)
    np.clip(idx, 0, dims - 1, out=idx)

    if min_points <= 1:
        grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    else:
        flat = (idx[:, 0] * dims[1] + idx[:, 1]) * dims[2] + idx[:, 2]
        counts = np.bincount(flat, minlength=int(np.prod(dims)))
        grid = (counts >= min_points).reshape(grid.shape)

    return VoxelOccupancy(origin=lo, resolution=resolution, grid=grid)
