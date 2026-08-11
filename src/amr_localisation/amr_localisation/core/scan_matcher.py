"""2D scan-to-scan matching by trimmed ICP.

This exists so the EKF has a LiDAR-derived velocity source. Wheel odometry on a
differential-drive AMR is excellent in yaw rate and poor in absolute yaw, and it
goes badly wrong exactly when it matters most — during a wheel slip on a wet
patch of warehouse floor, which is also when the robot is least likely to be
where it thinks. An IMU catches fast rotations but its yaw drifts. Scan matching
is uncorrelated with both: it fails on featureless corridors, where wheel
odometry is fine, and it is unbothered by slip, where wheel odometry is not.
Fusing the three covers each one's failure mode with the others.

Why ICP written out here rather than a package dependency: the two obvious
candidates, ``rf2o_laser_odometry`` and ``scan_tools/laser_scan_matcher``, are
both source-only on Humble. Vendoring a build of either into the Docker image to
obtain roughly 150 lines of arithmetic is a poor trade against writing the 150
lines, and a self-contained implementation can be unit tested for accuracy
directly, which is what ``tests/test_scan_matcher.py`` does.

The algorithm is textbook point-to-point ICP with two additions that matter in a
warehouse:

* **Trimmed correspondences.** Racking is highly repetitive. A plain
  nearest-neighbour association will happily match bay *n* to bay *n+1* and
  produce a confident, entirely wrong 2.7 m translation. Keeping only the best
  ``trim_ratio`` of correspondences by residual, re-evaluated every iteration,
  makes the estimate robust to that whole class of mismatch.
* **A reported fitness.** The matcher says how well it did — inlier fraction and
  residual — and the node refuses to publish when fitness is poor rather than
  emitting a confident bad velocity into the EKF.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class MatchResult:
    """Outcome of aligning a source scan onto a reference scan."""

    dx: float
    dy: float
    dtheta: float
    converged: bool
    iterations: int
    inlier_count: int
    total_points: int
    mean_residual: float

    @property
    def fitness(self) -> float:
        """Fraction of source points within ``inlier_distance`` of the reference.

        Note what this is *not*: it is not the fraction of points that found a
        correspondence at all. That number is useless as a quality signal,
        because trimming caps it at ``trim_ratio`` by construction and, in a
        dense scan, almost every point finds some neighbour within the
        correspondence cutoff no matter how wrong the transform is. Measuring
        against a tight inlier distance instead makes a good match score near
        1.0 and a misaligned one score far lower, which is the discrimination
        the caller's gate depends on.
        """
        return self.inlier_count / self.total_points if self.total_points else 0.0

    @property
    def pose(self) -> Tuple[float, float, float]:
        return (self.dx, self.dy, self.dtheta)

    def as_dict(self) -> dict:
        return {
            "dx": self.dx,
            "dy": self.dy,
            "dtheta": self.dtheta,
            "converged": self.converged,
            "iterations": self.iterations,
            "inlier_count": self.inlier_count,
            "total_points": self.total_points,
            "mean_residual": self.mean_residual,
            "fitness": self.fitness,
        }


def scan_to_points(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.05,
    range_max: float = 30.0,
) -> np.ndarray:
    """Convert a ``LaserScan`` range array into an (N, 2) cartesian cloud.

    Returns outside ``[range_min, range_max]``, and non-finite returns, are
    dropped. A LiDAR reports a no-return as ``inf`` (or, on some drivers, as
    ``range_max`` exactly), and carrying those through puts a phantom wall at
    the sensor's maximum range in every direction.
    """
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + np.arange(len(ranges)) * angle_increment

    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def _solve_se2(source: np.ndarray, target: np.ndarray) -> Tuple[float, float, float]:
    """Closed-form least-squares SE(2) alignment of paired points.

    The 2D case of Kabsch/Umeyama. There is no iteration and no linearisation
    here: for a fixed set of correspondences this is the exact minimiser, which
    is why ICP converges as fast as it does.
    """
    mu_s = source.mean(axis=0)
    mu_t = target.mean(axis=0)
    s = source - mu_s
    t = target - mu_t

    num = float(np.sum(s[:, 0] * t[:, 1] - s[:, 1] * t[:, 0]))
    den = float(np.sum(s[:, 0] * t[:, 0] + s[:, 1] * t[:, 1]))
    theta = math.atan2(num, den)

    c, s_ = math.cos(theta), math.sin(theta)
    tx = mu_t[0] - (c * mu_s[0] - s_ * mu_s[1])
    ty = mu_t[1] - (s_ * mu_s[0] + c * mu_s[1])
    return float(tx), float(ty), float(theta)


def _apply(points: np.ndarray, pose: Tuple[float, float, float]) -> np.ndarray:
    tx, ty, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    return points @ np.array([[c, s], [-s, c]]) + np.array([tx, ty])


def _compose(a: Tuple[float, float, float], b: Tuple[float, float, float]):
    """Compose ``a`` then ``b`` in SE(2)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(bt), math.sin(bt)
    return (bx + c * ax - s * ay, by + s * ax + c * ay, at + bt)


def match_scans(
    source: np.ndarray,
    reference: np.ndarray,
    initial_guess: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    max_correspondence_distance: float = 0.5,
    max_iterations: int = 30,
    tolerance: float = 1e-4,
    trim_ratio: float = 0.80,
    min_points: int = 30,
    inlier_distance: float = 0.05,
) -> Optional[MatchResult]:
    """Estimate the rigid transform taking ``source`` onto ``reference``.

    Both clouds are (N, 2) in the sensor frame. The returned pose is the motion
    of the sensor between the two scans, expressed in the reference scan's frame.

    Parameters
    ----------
    initial_guess
        Seed from wheel odometry. ICP has a basin of convergence of roughly half
        the smallest repeating feature spacing — 1.35 m here, given racking bays
        at 2.7 m — so a seed is not optional at speed. At 1.0 m/s and 10 Hz the
        robot moves 0.10 m per scan, which is inside the basin unseeded, but a
        rotation in place is not: 1.0 rad/s puts the far end of a 15 m scan 1.5 m
        away between frames.
    max_correspondence_distance
        Correspondences further apart than this are discarded outright before
        trimming. Set from the largest expected inter-scan motion plus the
        largest expected point spacing.
    trim_ratio
        Fraction of surviving correspondences, ordered by residual, used in the
        solve. 0.80 tolerates the 10-20% of a warehouse scan that is genuinely
        unmatched between frames: newly revealed aisle behind a passed upright,
        and points that fell off the far end of the range.
    min_points
        Below this the geometry is too weak to constrain three degrees of
        freedom and ``None`` is returned rather than a guess.
    inlier_distance
        Residual, in metres, below which a source point counts towards
        ``fitness``. 0.05 m is five times the sensor's 0.01 m range noise, so a
        correctly aligned scan puts essentially every point inside it while a
        misaligned one does not. This is the parameter that makes ``fitness`` a
        usable rejection signal; see the note on that property.

    Returns
    -------
    ``None`` if either cloud is too small to match. Otherwise a
    :class:`MatchResult`, whose ``converged`` and ``fitness`` fields the caller
    is expected to check before using the pose.
    """
    source = np.asarray(source, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    if len(source) < min_points or len(reference) < min_points:
        return None
    if not (0.0 < trim_ratio <= 1.0):
        raise ValueError("trim_ratio must be in (0, 1], got %r" % (trim_ratio,))

    tree = cKDTree(reference)
    pose = tuple(float(v) for v in initial_guess)

    converged = False
    iterations = 0

    for iterations in range(1, max_iterations + 1):
        moved = _apply(source, pose)
        distances, indices = tree.query(moved, k=1, distance_upper_bound=max_correspondence_distance)

        # cKDTree flags a miss with distance inf and index == len(reference).
        valid = np.isfinite(distances)
        n_valid = int(valid.sum())
        if n_valid < min_points:
            break

        d_valid = distances[valid]
        if trim_ratio < 1.0:
            keep_n = max(min_points, int(round(n_valid * trim_ratio)))
            if keep_n < n_valid:
                cutoff = np.partition(d_valid, keep_n - 1)[keep_n - 1]
                trim_mask = valid & (distances <= cutoff)
            else:
                trim_mask = valid
        else:
            trim_mask = valid

        src_sel = moved[trim_mask]
        tgt_sel = reference[indices[trim_mask]]
        if len(src_sel) < min_points:
            break

        delta = _solve_se2(src_sel, tgt_sel)
        pose = _compose(pose, delta)

        step = math.hypot(delta[0], delta[1]) + abs(delta[2])
        if step < tolerance:
            converged = True
            break

    # Final scoring pass at the pose actually being returned. Scoring from the
    # last iteration's distances instead would describe the pose from *before*
    # that iteration's update, which is exactly the situation where the two
    # differ most: a match that is still moving when it hits the iteration cap.
    final_distances, _ = tree.query(
        _apply(source, pose), k=1, distance_upper_bound=max_correspondence_distance
    )
    matched = np.isfinite(final_distances)
    inlier_count = int((matched & (final_distances <= inlier_distance)).sum())

    # Mean residual is reported over the same trimmed fraction the solver
    # optimised, not over every correspondence. Averaging in the worst 20% would
    # mostly measure how much of the scene is legitimately unshared between the
    # two scans — newly revealed aisle, points fallen off the far end of range —
    # which varies with robot speed and says nothing about alignment quality.
    if matched.any():
        d_matched = final_distances[matched]
        keep_n = max(1, int(round(len(d_matched) * trim_ratio)))
        mean_residual = float(np.partition(d_matched, keep_n - 1)[:keep_n].mean())
    else:
        mean_residual = float("inf")

    theta = (pose[2] + math.pi) % (2.0 * math.pi) - math.pi
    return MatchResult(
        dx=float(pose[0]),
        dy=float(pose[1]),
        dtheta=float(theta),
        converged=converged,
        iterations=iterations,
        inlier_count=inlier_count,
        total_points=len(source),
        mean_residual=mean_residual,
    )
