"""Keyframe-based LiDAR odometry, independent of ROS.

The keyframe policy lives here rather than in the node so that the offline
benchmark in ``analysis/scan_odometry_benchmark.py`` measures the same code that
runs on the robot. A benchmark that reimplements the policy it is measuring will
eventually disagree with it, and the disagreement will be discovered by the
robot rather than by the benchmark.

WHY KEYFRAMES

Matching each scan against its immediate predecessor accumulates the per-match
error on every frame. At 10 Hz that is 10 error samples compounding per second
and the estimate drifts visibly within a minute. Holding a reference scan until
the robot has actually moved means error accumulates once per keyframe instead —
at 0.35 m and 0.6 m/s, roughly every 0.6 s rather than every 0.1 s — and the
residual within a keyframe interval does not accumulate at all, because each
match is against the same fixed reference.

The cost is that overlap falls as the robot moves away from the keyframe, so the
match gets harder. That is what bounds the keyframe distance from above.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from amr_common.transforms import se2_compose, se2_inverse

from .scan_matcher import MatchResult, match_scans

Pose = Tuple[float, float, float]


@dataclass
class OdometryUpdate:
    """One accepted scan-odometry update."""

    pose: Pose
    previous_pose: Pose
    match: MatchResult
    keyframed: bool

    @property
    def delta(self) -> Pose:
        """Motion since the previous accepted update, in the previous body frame."""
        return se2_compose(se2_inverse(*self.previous_pose), self.pose)


class ScanOdometry:
    """Integrates scan matches into a pose, taking keyframes as it goes."""

    def __init__(
        self,
        keyframe_distance: float = 0.35,
        keyframe_rotation: float = 0.35,
        min_fitness: float = 0.60,
        max_mean_residual: float = 0.030,
        require_converged: bool = True,
        retry_unseeded: bool = True,
        max_correspondence_distance: float = 0.50,
        max_iterations: int = 50,
        trim_ratio: float = 0.80,
        inlier_distance: float = 0.05,
        initial_pose: Pose = (0.0, 0.0, 0.0),
    ) -> None:
        self.keyframe_distance = keyframe_distance
        self.keyframe_rotation = keyframe_rotation
        self.min_fitness = min_fitness
        self.max_mean_residual = max_mean_residual
        self.require_converged = require_converged
        self.retry_unseeded = retry_unseeded
        self.max_correspondence_distance = max_correspondence_distance
        self.max_iterations = max_iterations
        self.trim_ratio = trim_ratio
        self.inlier_distance = inlier_distance

        self.pose: Pose = tuple(float(v) for v in initial_pose)
        self.matched = 0
        self.rejected = 0
        self.reseeded = 0

        self._ref_points: Optional[np.ndarray] = None
        self._ref_pose: Pose = self.pose
        self._ref_odom: Optional[Pose] = None

    # ------------------------------------------------------------------
    def _adopt_keyframe(self, points: np.ndarray, pose: Pose, odom_pose: Optional[Pose]) -> None:
        self._ref_points = points
        self._ref_pose = pose
        self._ref_odom = odom_pose

    def _match(self, points: np.ndarray, guess: Pose) -> Optional[MatchResult]:
        return match_scans(
            points,
            self._ref_points,
            initial_guess=guess,
            max_correspondence_distance=self.max_correspondence_distance,
            max_iterations=self.max_iterations,
            trim_ratio=self.trim_ratio,
            inlier_distance=self.inlier_distance,
        )

    def _initial_guess(self, odom_pose: Optional[Pose]) -> Pose:
        """Motion since the keyframe according to wheel odometry.

        Without a seed, ICP has to find a transform from a standing start, and
        its basin of convergence is roughly half the smallest repeating feature
        spacing — 1.35 m here, given 2.7 m racking bays. Translation at cruise
        stays inside that; a fast rotation does not, because 1 rad/s moves the
        far end of a 15 m scan by 1.5 m between frames.
        """
        if odom_pose is None or self._ref_odom is None:
            return (0.0, 0.0, 0.0)
        return se2_compose(se2_inverse(*self._ref_odom), odom_pose)

    def _acceptable(self, result: Optional[MatchResult]) -> bool:
        """Three independent gates, all of which must pass.

        Thresholds are measured, not assumed. ``tests/test_scan_matcher.py``
        reproduces the distributions they come from: over correct matches on
        this floorplan the trimmed mean residual never exceeds 0.023 m, so the
        0.030 m ceiling sits just outside the good population. The residual
        floor of ~0.018 m is set by the 0.5 degree beam spacing rather than by
        range noise — at 10 m one beam step is 87 mm, so nearest-neighbour
        distance is dominated by angular sampling.

        Fitness needs more care. Correct matches have a 5th percentile of 0.78
        but a long lower tail reaching 0.38, because how many points fall inside
        the inlier distance depends on how much of the scene the two scans
        actually share. A gate placed at the 5th percentile therefore throws
        away perfectly good matches — 0.70 was tried and rejected a correct
        0.10 m match that scored 0.671. 0.60 keeps those while still sitting
        above the median of incorrect matches (0.47), and the convergence and
        residual gates cover what it lets through.

        WHAT THESE GATES CANNOT DO, which matters more than what they can:
        a scan matched onto the *wrong racking bay* is geometrically perfect.
        It converges, its fitness reaches 1.0 and its residual is as low as a
        correct match, because one bay really does look exactly like the next.
        No local quality measure can separate those cases — the information is
        not present in the two scans. Rejecting them is AMCL's job, via the
        global map, and Cartographer's, via the pose graph. See
        docs/limitations.md.

        Any one of them alone lets a bad match through, and a bad match is far
        worse than a missing one: the pose is integrated, so a single accepted
        misalignment becomes a permanent offset that every subsequent keyframe
        inherits. All three are cheap and they fail on different things.

        * **Convergence.** ICP that hits its iteration cap is still moving and
          has not settled anywhere. It reports whatever pose it happened to be
          at, which is not an estimate of anything.
        * **Fitness.** Fraction of points within ``inlier_distance``. Catches a
          match that converged neatly onto the wrong structure — the racking
          aliasing case, where the answer is stable and wrong.
        * **Mean residual.** Absolute alignment quality in metres, independent
          of how many points happened to be in view. Catches the case where a
          sparse scan reports high fitness from very few points.
        """
        if result is None:
            return False
        if self.require_converged and not result.converged:
            return False
        if result.fitness < self.min_fitness:
            return False
        if result.mean_residual > self.max_mean_residual:
            return False
        return True

    def update(
        self, points: np.ndarray, odom_pose: Optional[Pose] = None
    ) -> Optional[OdometryUpdate]:
        """Process one scan. Returns ``None`` when no pose update was produced.

        ``None`` is returned both for the very first scan, which only
        establishes a keyframe, and for a rejected match. A caller must not
        treat it as "no motion" — it means "no information", and the right
        response is to leave the interval to the other odometry sources.
        """
        if self._ref_points is None:
            self._adopt_keyframe(points, self.pose, odom_pose)
            return None

        guess = self._initial_guess(odom_pose)
        result = self._match(points, guess)

        # If a seeded match fails, retry from zero before giving up.
        #
        # This is not belt-and-braces, it addresses a specific and slightly
        # perverse failure. The seed comes from wheel odometry, and the headline
        # reason this scan matcher exists at all is to cover wheel slip — so the
        # moment the seed is most wrong is precisely the moment the matcher is
        # most needed. A 0.25 m phantom translation from a slipping drive wheel
        # is enough to pull ICP into the neighbouring racking bay, where it
        # converges cleanly onto the wrong answer. Retrying unseeded costs one
        # extra match on an already-rare path and recovers most of those cases.
        if not self._acceptable(result) and guess != (0.0, 0.0, 0.0) and self.retry_unseeded:
            retry = self._match(points, (0.0, 0.0, 0.0))
            if self._acceptable(retry):
                self.reseeded += 1
                result = retry

        if not self._acceptable(result):
            self.rejected += 1
            # Whatever made this scan unmatchable against the old reference will
            # keep doing so, so take a fresh keyframe rather than retrying.
            #
            # The keyframe is anchored at the last *good* pose, not at a guess
            # about where the robot is now. The motion during the rejected
            # interval is genuinely unknown to this estimator and is left to the
            # EKF's other sources; inventing it here would launder wheel
            # odometry through a topic that claims to be LiDAR-derived.
            self._adopt_keyframe(points, self.pose, odom_pose)
            return None

        self.matched += 1
        previous = self.pose
        self.pose = se2_compose(self._ref_pose, result.pose)

        keyframed = (
            math.hypot(result.dx, result.dy) >= self.keyframe_distance
            or abs(result.dtheta) >= self.keyframe_rotation
        )
        if keyframed:
            self._adopt_keyframe(points, self.pose, odom_pose)

        return OdometryUpdate(
            pose=self.pose, previous_pose=previous, match=result, keyframed=keyframed
        )

    @property
    def reject_rate(self) -> float:
        total = self.matched + self.rejected
        return self.rejected / total if total else 0.0
