"""A synthetic 2D LiDAR over a line-segment floorplan.

Used to measure the scan matcher's accuracy, and to give the localisation tests
a warehouse whose repeating structure is the exact thing that breaks naive ICP.
The floorplan mirrors ``simulation/worlds/warehouse.world``: a long aisle
flanked by racking bays at a fixed 2.7 m pitch. That regularity is the point —
matched against itself with no initial guess, this scene will pull an
unprotected ICP a whole bay out of position, and the trimming and seeding in
``scan_matcher`` are what stop it.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

Segment = Tuple[float, float, float, float]  # x1, y1, x2, y2


def ray_segment_distances(
    origin: np.ndarray,
    angles: np.ndarray,
    segments: Sequence[Segment],
    max_range: float,
) -> np.ndarray:
    """Nearest intersection distance along each ray, ``inf`` where nothing is hit.

    Solves ``origin + t * d == p1 + u * (p2 - p1)`` for every ray against every
    segment, vectorised over rays.
    """
    dirs = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    best = np.full(len(angles), np.inf)

    for x1, y1, x2, y2 in segments:
        seg = np.array([x2 - x1, y2 - y1])
        p1 = np.array([x1, y1])

        # 2D cross product of the ray direction with the segment direction. Zero
        # means parallel, so no unique intersection.
        denom = dirs[:, 0] * seg[1] - dirs[:, 1] * seg[0]
        parallel = np.abs(denom) < 1e-12
        safe = np.where(parallel, 1.0, denom)

        rel = p1 - origin
        t = (rel[0] * seg[1] - rel[1] * seg[0]) / safe
        u = (rel[0] * dirs[:, 1] - rel[1] * dirs[:, 0]) / safe

        hit = (~parallel) & (t > 0) & (t < best) & (u >= 0.0) & (u <= 1.0)
        best = np.where(hit, t, best)

    return np.where(best <= max_range, best, np.inf)


def simulate_2d_scan(
    segments: Sequence[Segment],
    pose: Tuple[float, float, float],
    n_beams: int = 720,
    angle_min: float = -math.pi,
    angle_max: float = math.pi,
    max_range: float = 25.0,
    range_sigma: float = 0.01,
    dropout: float = 0.0,
    seed: int = 0,
) -> np.ndarray:
    """One 2D scan from ``pose = (x, y, theta)``, returned as (N, 2) sensor-frame points.

    ``range_sigma`` of 0.01 m matches the ``gaussianNoise`` set on the 2D ray
    sensor in ``amr_description``.
    """
    x, y, theta = pose
    origin = np.array([x, y], dtype=np.float64)
    angles_local = np.linspace(angle_min, angle_max, n_beams, endpoint=False)
    ranges = ray_segment_distances(origin, angles_local + theta, segments, max_range)

    finite = np.isfinite(ranges)
    rng = np.random.default_rng(seed)
    if dropout > 0:
        finite &= rng.random(len(ranges)) >= dropout

    r = ranges[finite]
    a = angles_local[finite]
    if range_sigma > 0:
        r = r + rng.normal(0.0, range_sigma, size=len(r))

    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def _box(cx: float, cy: float, w: float, h: float) -> List[Segment]:
    hw, hh = w / 2.0, h / 2.0
    x0, x1 = cx - hw, cx + hw
    y0, y1 = cy - hh, cy + hh
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def warehouse_floorplan(
    length: float = 30.0,
    width: float = 12.0,
    aisle_half_width: float = 1.4,
    bay_pitch: float = 2.7,
    n_bays: int = 10,
    obstacles: Optional[Sequence[Tuple[float, float, float, float]]] = None,
) -> List[Segment]:
    """Segments for the aisle: perimeter walls, racking uprights, aisle obstacles.

    Note the deliberate absence of any feature that breaks the bay periodicity
    along the aisle. A floorplan with a distinctive landmark every few metres
    makes scan matching look far better than it is in a real warehouse.
    """
    half_l = length / 2.0
    half_w = width / 2.0

    segments: List[Segment] = [
        (-half_l, -half_w, half_l, -half_w),
        (half_l, -half_w, half_l, half_w),
        (half_l, half_w, -half_l, half_w),
        (-half_l, half_w, -half_l, -half_w),
    ]

    for side in (-1.0, 1.0):
        for depth_offset in (0.0, 0.9):
            y = side * (aisle_half_width + depth_offset)
            for bay in range(n_bays + 1):
                x = -half_l + 1.5 + bay * bay_pitch
                if x > half_l - 1.0:
                    break
                segments.extend(_box(x, y, 0.10, 0.10))

    if obstacles is None:
        obstacles = [(4.0, 0.35, 1.2, 0.8), (-6.5, -0.40, 1.2, 0.8)]
    for cx, cy, w, h in obstacles:
        segments.extend(_box(cx, cy, w, h))

    return segments
