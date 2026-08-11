"""Ground-plane removal for a warehouse 3D LiDAR.

Plain RANSAC over the whole cloud is the obvious approach and it is the wrong
one here. A warehouse scan is dominated by long flat surfaces that are *not* the
floor: the tops of pallet racking, the flat faces of stacked cartons, and, if
the sensor sits low, the underside of a shelf. Given enough iterations RANSAC
will happily lock onto a shelf and declare it the ground.

Three constraints fix that, and all three are cheap:

1. **Seed band.** Sample the three hypothesis points only from returns that lie
   near the expected floor height. The sensor's mounting height above the floor
   is known to a few centimetres on any real AMR, so this costs nothing in
   generality.
2. **Normal constraint.** Reject any hypothesis whose normal is more than
   ``max_tilt_deg`` from vertical. Shelf faces are vertical and are rejected
   outright; shelf tops survive this test, which is why the seed band is needed
   as well.
3. **Least-squares refit.** The three-point hypothesis that wins the vote is
   noisy. Refit the plane to its inliers by SVD and re-threshold once. This
   typically tightens the residual spread by a third and matters because the
   threshold is what decides whether a 4 cm pallet lip is ground or object.

Scoring is always done against the whole cloud, never against the seed band, so
the winning plane is the one that explains the most returns overall.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Hypotheses scored per batch. Large enough to amortise numpy's per-call
# overhead, small enough that the adaptive stopping rule still bites and that
# the (n_score x _CHUNK) residual block stays inside L2.
_CHUNK = 64


@dataclass(frozen=True)
class GroundPlane:
    """A fitted ground plane in the form ``normal . p + d = 0``."""

    normal: np.ndarray
    d: float
    inlier_count: int
    total_points: int
    iterations_used: int
    rms_residual: float

    @property
    def inlier_ratio(self) -> float:
        return self.inlier_count / self.total_points if self.total_points else 0.0

    @property
    def tilt_deg(self) -> float:
        """Angle between the plane normal and vertical, in degrees."""
        return math.degrees(math.acos(min(1.0, abs(float(self.normal[2])))))

    def height_above(self, points: np.ndarray) -> np.ndarray:
        """Signed perpendicular distance from the plane, positive upwards."""
        points = np.asarray(points, dtype=np.float64)
        return points @ self.normal + self.d

    def as_dict(self) -> dict:
        return {
            "normal": [float(v) for v in self.normal],
            "d": float(self.d),
            "inlier_count": int(self.inlier_count),
            "total_points": int(self.total_points),
            "inlier_ratio": float(self.inlier_ratio),
            "iterations_used": int(self.iterations_used),
            "rms_residual": float(self.rms_residual),
            "tilt_deg": float(self.tilt_deg),
        }


def _fit_plane_svd(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """Total-least-squares plane fit. Returns a unit normal and offset ``d``."""
    centroid = points.mean(axis=0)
    centred = points - centroid
    # The singular vector for the smallest singular value is the plane normal.
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    normal = vh[2]
    norm = np.linalg.norm(normal)
    if norm < 1e-12:
        raise np.linalg.LinAlgError("degenerate plane fit")
    normal = normal / norm
    if normal[2] < 0:
        normal = -normal
    d = float(-normal @ centroid)
    return normal, d


def _required_iterations(inlier_ratio: float, confidence: float, sample: int = 3) -> float:
    """Standard RANSAC stopping rule for the number of trials still needed."""
    if inlier_ratio <= 0.0:
        return math.inf
    p_all_inliers = inlier_ratio ** sample
    if p_all_inliers >= 1.0:
        return 0.0
    denom = math.log(max(1e-12, 1.0 - p_all_inliers))
    return math.log(max(1e-12, 1.0 - confidence)) / denom


def fit_ground_plane(
    points: np.ndarray,
    distance_threshold: float = 0.06,
    max_tilt_deg: float = 10.0,
    max_iterations: int = 300,
    confidence: float = 0.999,
    seed_band: Optional[Tuple[float, float]] = (-0.50, 0.35),
    min_inlier_ratio: float = 0.10,
    refine: bool = True,
    score_subsample: Optional[int] = 4000,
    seed: int = 0,
) -> Optional[GroundPlane]:
    """Fit the floor plane. Returns ``None`` if no plane clears the constraints.

    Parameters
    ----------
    distance_threshold
        Perpendicular distance, in metres, inside which a point counts as
        ground. 0.06 m is set from the simulated LiDAR's range noise
        (sigma = 0.01 m, so 6 sigma) plus a margin for the floor's own
        flatness. Tightening it below ~0.04 m starts shaving the outer rings
        off the ground return, where incidence angle inflates range error.
    max_tilt_deg
        Reject hypotheses whose normal is further than this from vertical. The
        warehouse floor is flat, but the sensor pitches by a degree or two under
        acceleration, so this is not zero.
    seed_band
        ``(z_min, z_max)`` in sensor frame from which hypothesis points are
        drawn. ``None`` disables the band and samples the whole cloud.
    min_inlier_ratio
        A plane explaining less than this fraction of the cloud is treated as a
        failed fit rather than a bad floor. The caller then keeps the whole
        cloud as non-ground, which is the safe direction to fail in: an
        unremoved floor produces spurious obstacles, never missing ones.
    score_subsample
        Number of points used to *score* each hypothesis. Scoring every
        hypothesis against all ~28k returns is where a naive RANSAC spends
        essentially all of its time, and it buys nothing: the inlier ratio of a
        plane is estimated to well under a percent from a few thousand samples.
        The winning plane is always re-evaluated against the full cloud before
        it is returned, so the output is exact. Set to ``None`` to score against
        everything.
    seed
        Fixed by default. Segmentation output feeds a safety-adjacent zone
        publisher, and a non-deterministic front end makes any downstream
        regression impossible to bisect.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3), got %r" % (points.shape,))
    n = len(points)
    if n < 3:
        return None

    if seed_band is not None:
        lo, hi = seed_band
        seed_mask = (points[:, 2] >= lo) & (points[:, 2] <= hi)
        seed_idx = np.flatnonzero(seed_mask)
        if len(seed_idx) < 3:
            seed_idx = np.arange(n)
    else:
        seed_idx = np.arange(n)

    rng = np.random.default_rng(seed)
    cos_limit = math.cos(math.radians(max_tilt_deg))

    if score_subsample is not None and n > score_subsample:
        score_pts = points[rng.choice(n, size=score_subsample, replace=False)]
    else:
        score_pts = points
    n_score = len(score_pts)

    # Draw every hypothesis triplet up front. Calling rng.choice(replace=False)
    # once per iteration dominates the runtime of this function, because for a
    # sample of 3 out of several thousand candidates numpy still pays for the
    # machinery of a without-replacement draw. Sampling with replacement and
    # letting the degenerate-triangle check below reject the occasional repeat
    # is both faster and statistically equivalent at these ratios.
    sample_idx = seed_idx[rng.integers(0, len(seed_idx), size=(max_iterations, 3))]

    best_normal: Optional[np.ndarray] = None
    best_d = 0.0
    best_score = 0
    iterations_used = 0
    budget = float(max_iterations)

    # Hypotheses are evaluated in chunks rather than one at a time. Scoring a
    # single plane against 4000 points is a few microseconds of arithmetic
    # wrapped in a few tens of microseconds of numpy call overhead, so a
    # one-at-a-time loop is roughly 90% overhead. Scoring a whole chunk is one
    # (n_score x chunk) matmul. The chunking, rather than one batch over all
    # iterations, is what preserves the adaptive stopping rule: the budget is
    # re-checked between chunks, so a scene with an obvious floor still exits
    # early instead of grinding through every iteration.
    for start in range(0, max_iterations, _CHUNK):
        if start >= budget:
            break
        stop = min(start + _CHUNK, max_iterations, int(math.ceil(budget)))
        if stop <= start:
            break

        tri = points[sample_idx[start:stop]]  # (m, 3, 3)
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        norms = np.linalg.norm(normals, axis=1)

        valid = norms > 1e-9  # reject collinear / repeated samples
        normals = normals / np.where(valid, norms, 1.0)[:, None]
        flip = normals[:, 2] < 0
        normals[flip] = -normals[flip]
        valid &= normals[:, 2] >= cos_limit  # too tilted to be a warehouse floor

        if valid.any():
            ds = -np.einsum("ij,ij->i", normals, tri[:, 0])
            residual = np.abs(score_pts @ normals.T + ds[None, :])
            scores = (residual <= distance_threshold).sum(axis=0)
            scores = np.where(valid, scores, -1)

            k = int(np.argmax(scores))
            if scores[k] > best_score:
                best_score = int(scores[k])
                best_normal = normals[k].copy()
                best_d = float(ds[k])
                budget = min(budget, _required_iterations(best_score / n_score, confidence))

        iterations_used = stop

    if best_normal is None:
        return None
    if best_score / n_score < min_inlier_ratio:
        return None

    # Re-evaluate the winner against the whole cloud so the returned mask and
    # counts are exact rather than subsample estimates.
    best_inliers = np.abs(points @ best_normal + best_d) <= distance_threshold
    best_count = int(best_inliers.sum())
    if best_count < 3:
        return None

    normal, d = _fit_plane_svd(points[best_inliers])

    if refine:
        residual = np.abs(points @ normal + d)
        refined = residual <= distance_threshold
        if int(refined.sum()) >= 3:
            try:
                normal, d = _fit_plane_svd(points[refined])
            except np.linalg.LinAlgError:
                pass
            best_inliers = np.abs(points @ normal + d) <= distance_threshold
            best_count = int(best_inliers.sum())

    # The refit can, rarely, tip the plane past the tilt limit. Refuse it rather
    # than return a plane that violates a constraint the caller relies on.
    if normal[2] < cos_limit:
        return None

    rms = float(np.sqrt(np.mean((points[best_inliers] @ normal + d) ** 2))) if best_count else 0.0

    return GroundPlane(
        normal=normal,
        d=d,
        inlier_count=best_count,
        total_points=n,
        iterations_used=iterations_used,
        rms_residual=rms,
    )


def remove_ground(
    points: np.ndarray,
    distance_threshold: float = 0.06,
    max_height: Optional[float] = 3.0,
    **kwargs,
) -> Tuple[np.ndarray, np.ndarray, Optional[GroundPlane]]:
    """Split ``points`` into ``(ground_mask, object_mask, plane)``.

    ``max_height`` additionally discards returns more than that far above the
    fitted plane. In this warehouse the AMR is 0.4 m tall and the tallest thing
    it must avoid is a 2.2 m racking upright, so anything above 3 m is roof
    structure, lighting gantry or sprinkler pipe. Keeping it would push
    permanent phantom obstacles into the 3D occupancy grid. Set to ``None`` to
    keep the full column.

    Failing to fit returns an all-false ground mask, i.e. everything is treated
    as an obstacle.
    """
    points = np.asarray(points, dtype=np.float64)
    n = len(points)
    plane = fit_ground_plane(points, distance_threshold=distance_threshold, **kwargs)

    if plane is None:
        return np.zeros(n, dtype=bool), np.ones(n, dtype=bool), None

    height = plane.height_above(points)
    ground = np.abs(height) <= distance_threshold
    objects = ~ground
    if max_height is not None:
        objects &= height <= max_height

    return ground, objects, plane
