"""2D scan matching and keyframe odometry.

The accuracy figures asserted here are the source of the numbers quoted in
docs/localisation-accuracy.md and of the rejection thresholds hard-coded in
``ScanOdometry``. They are bounds with headroom, not exact values, so ordinary
noise does not turn the suite red — but they are tight enough that a real
regression breaks them.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from amr_common.transforms import se2_compose, se2_inverse, wrap_angle
from amr_localisation.core.scan_matcher import match_scans, scan_to_points
from amr_localisation.core.scan_odometry import ScanOdometry
from amr_localisation.core.synthetic2d import simulate_2d_scan

BAY_PITCH = 2.7


def _relative(pose_a, pose_b):
    """Pose of b expressed in a's frame."""
    return se2_compose(se2_inverse(*pose_a), pose_b)


def _error(result, truth):
    return (
        math.hypot(result.dx - truth[0], result.dy - truth[1]),
        abs(wrap_angle(result.dtheta - truth[2])),
    )


class TestScanToPoints:
    def test_converts_polar_to_cartesian(self):
        ranges = np.array([1.0, 1.0, 1.0, 1.0])
        points = scan_to_points(ranges, -math.pi, math.pi / 2)
        assert len(points) == 4
        assert np.allclose(np.linalg.norm(points, axis=1), 1.0)

    def test_drops_out_of_range_and_non_finite(self):
        ranges = np.array([np.inf, np.nan, 0.01, 100.0, 5.0])
        points = scan_to_points(ranges, 0.0, 0.1, range_min=0.05, range_max=30.0)
        assert len(points) == 1

    def test_no_return_does_not_become_a_phantom_wall(self):
        """inf must be dropped, not treated as max range."""
        ranges = np.full(100, np.inf)
        assert len(scan_to_points(ranges, -math.pi, 0.0628)) == 0


class TestMatchAccuracy:
    @pytest.mark.parametrize("dx,dtheta", [(0.05, 0.0), (0.10, 0.05), (0.0, 0.10), (0.20, -0.08)])
    def test_recovers_known_motion(self, floorplan, dx, dtheta):
        a_pose = (-6.0, 0.1, 0.05)
        b_pose = se2_compose(a_pose, (dx, 0.0, dtheta))
        a = simulate_2d_scan(floorplan, a_pose, seed=11)
        b = simulate_2d_scan(floorplan, b_pose, seed=12)

        result = match_scans(b, a, initial_guess=(dx, 0.0, dtheta))
        assert result is not None
        trans_err, rot_err = _error(result, _relative(a_pose, b_pose))
        assert trans_err < 0.03
        assert math.degrees(rot_err) < 1.0

    def test_accuracy_distribution_over_the_aisle(self, floorplan):
        """The measured per-match error the covariance model is built on."""
        rng = np.random.default_rng(0)
        trans, rot, fitness, residual = [], [], [], []

        for trial in range(30):
            x = rng.uniform(-11.0, 11.0)
            a_pose = (x, rng.uniform(-0.4, 0.4), rng.uniform(-0.3, 0.3))
            step = (rng.uniform(0.02, 0.12), 0.0, rng.uniform(-0.08, 0.08))
            b_pose = se2_compose(a_pose, step)

            a = simulate_2d_scan(floorplan, a_pose, seed=100 + trial)
            b = simulate_2d_scan(floorplan, b_pose, seed=200 + trial)
            result = match_scans(b, a, initial_guess=step)
            assert result is not None

            t_err, r_err = _error(result, _relative(a_pose, b_pose))
            trans.append(t_err)
            rot.append(r_err)
            fitness.append(result.fitness)
            residual.append(result.mean_residual)

        trans = np.array(trans)
        rot = np.degrees(np.array(rot))
        assert np.median(trans) < 0.02, "median translation error %.4f m" % np.median(trans)
        assert np.percentile(trans, 90) < 0.06
        assert np.median(rot) < 0.6, "median rotation error %.3f deg" % np.median(rot)

        # These bound the thresholds in ScanOdometry. If a change pushes good
        # matches below the fitness floor or above the residual ceiling, the
        # gates start rejecting healthy data and this catches it here rather
        # than as a mysteriously high reject rate on the robot.
        assert np.percentile(fitness, 5) > 0.60
        assert np.max(residual) < 0.030

    def test_reports_failure_on_too_few_points(self, floorplan):
        a = simulate_2d_scan(floorplan, (0.0, 0.0, 0.0), seed=1)
        assert match_scans(np.zeros((5, 2)), a) is None

    def test_invalid_trim_ratio_rejected(self, floorplan):
        a = simulate_2d_scan(floorplan, (0.0, 0.0, 0.0), seed=1)
        with pytest.raises(ValueError, match="trim_ratio"):
            match_scans(a, a, trim_ratio=0.0)

    def test_identical_scans_give_identity(self, floorplan):
        a = simulate_2d_scan(floorplan, (0.0, 0.0, 0.0), seed=1)
        result = match_scans(a, a)
        assert result is not None
        assert result.converged
        assert abs(result.dx) < 1e-3 and abs(result.dy) < 1e-3 and abs(result.dtheta) < 1e-3
        assert result.fitness > 0.95


class TestAliasing:
    def test_a_bay_offset_seed_lands_a_bay_out_but_is_caught(self, floorplan):
        """A bad seed does pull the match a whole bay out — and is detected.

        Two things are being pinned down here, and the second was a surprise.

        First, ICP really does follow a bad seed into the neighbouring bay: seed
        it 2.7 m out and it settles 2.77 m from the truth, not back at the
        correct answer. That is the failure the ``retry_unseeded`` path exists
        for.

        Second, in *this* warehouse the wrong bay is detectable. The aisle is
        only about ten bays long, and the end walls, the roof columns and the
        pallets left in the aisle all break the periodicity, so a bay-offset
        match cannot satisfy the whole scan at once: fitness drops to about 0.58
        against a 0.60 gate, and ICP does not converge at all.

        That is a property of this floorplan, not a general guarantee. In a
        longer or more uniform aisle the wrong bay becomes genuinely
        indistinguishable, and only AMCL's global map can resolve it. See
        docs/limitations.md.
        """
        a_pose = (-4.0, 0.0, 0.0)
        b_pose = se2_compose(a_pose, (0.08, 0.0, 0.0))
        a = simulate_2d_scan(floorplan, a_pose, seed=21)
        b = simulate_2d_scan(floorplan, b_pose, seed=22)

        result = match_scans(b, a, initial_guess=(BAY_PITCH, 0.0, 0.0))
        assert result is not None
        trans_err, _ = _error(result, _relative(a_pose, b_pose))
        assert trans_err > 1.0, "expected the bad seed to pull the match a bay out"

        odometry = ScanOdometry()
        assert not odometry._acceptable(result), (
            "a bay-offset match passed the gates: fitness %.3f, residual %.4f, converged %s"
            % (result.fitness, result.mean_residual, result.converged)
        )

    def test_retry_unseeded_recovers_from_a_poisoned_seed(self, floorplan):
        """Wheel slip poisons the seed exactly when the matcher is most needed."""
        truth = (-6.0, 0.0, 0.0)
        odometry = ScanOdometry(retry_unseeded=True, initial_pose=truth)
        odometry.update(simulate_2d_scan(floorplan, truth, seed=31), odom_pose=truth)

        moved = se2_compose(truth, (0.10, 0.0, 0.0))
        # Wheel odometry reports a phantom 1.5 m jump, over half a bay pitch.
        poisoned = se2_compose(truth, (1.50, 0.0, 0.0))
        update = odometry.update(simulate_2d_scan(floorplan, moved, seed=32), odom_pose=poisoned)

        assert update is not None, "the poisoned seed was not recovered from"
        error = math.hypot(odometry.pose[0] - moved[0], odometry.pose[1] - moved[1])
        assert error < 0.10, "recovered pose is %.3f m from truth" % error


class TestScanOdometry:
    def test_first_scan_only_establishes_a_keyframe(self, floorplan):
        odometry = ScanOdometry()
        scan = simulate_2d_scan(floorplan, (0.0, 0.0, 0.0), seed=1)
        assert odometry.update(scan) is None

    def test_tracks_a_short_traverse(self, floorplan):
        odometry = ScanOdometry(initial_pose=(-8.0, 0.0, 0.0))
        truth = (-8.0, 0.0, 0.0)
        for step in range(25):
            scan = simulate_2d_scan(floorplan, truth, seed=300 + step)
            odometry.update(scan, odom_pose=truth)
            truth = se2_compose(truth, (0.06, 0.0, 0.0))

        error = math.hypot(odometry.pose[0] - truth[0], odometry.pose[1] - truth[1])
        assert error < 0.15, "drifted %.3f m over 1.5 m of travel" % error
        assert odometry.reject_rate < 0.15

    def test_keyframes_are_taken_on_distance(self, floorplan):
        odometry = ScanOdometry(keyframe_distance=0.20)
        truth = (-8.0, 0.0, 0.0)
        keyframes = 0
        for step in range(20):
            scan = simulate_2d_scan(floorplan, truth, seed=400 + step)
            update = odometry.update(scan, odom_pose=truth)
            if update is not None and update.keyframed:
                keyframes += 1
            truth = se2_compose(truth, (0.08, 0.0, 0.0))
        assert keyframes >= 4, "expected a keyframe roughly every 0.20 m"

    def test_non_converged_match_is_rejected(self, floorplan):
        """Regression guard.

        An earlier version computed `converged` and never looked at it. A match
        that hit the iteration cap while still moving was accepted, a keyframe
        was taken at the resulting pose, and the estimate carried a permanent
        35 degree yaw error for the rest of the run — while reporting healthy
        fitness, because every subsequent match was consistent with the wrong
        keyframe.
        """
        odometry = ScanOdometry(require_converged=True)
        assert odometry._acceptable(_FakeMatch(converged=False, fitness=0.99, residual=0.001)) is False
        assert odometry._acceptable(_FakeMatch(converged=True, fitness=0.99, residual=0.001)) is True

    def test_each_gate_rejects_independently(self, floorplan):
        odometry = ScanOdometry(min_fitness=0.60, max_mean_residual=0.030, require_converged=True)
        assert not odometry._acceptable(None)
        assert not odometry._acceptable(_FakeMatch(True, 0.50, 0.001)), "fitness gate"
        assert not odometry._acceptable(_FakeMatch(True, 0.99, 0.100)), "residual gate"
        assert not odometry._acceptable(_FakeMatch(False, 0.99, 0.001)), "convergence gate"
        assert odometry._acceptable(_FakeMatch(True, 0.99, 0.001))

    def test_rejection_holds_the_pose_rather_than_guessing(self, floorplan):
        """A rejected interval must not be filled in from wheel odometry."""
        odometry = ScanOdometry(min_fitness=1.01)  # reject everything
        scan = simulate_2d_scan(floorplan, (0.0, 0.0, 0.0), seed=1)
        odometry.update(scan, odom_pose=(0.0, 0.0, 0.0))
        before = odometry.pose
        moved = simulate_2d_scan(floorplan, (1.0, 0.0, 0.0), seed=2)
        assert odometry.update(moved, odom_pose=(1.0, 0.0, 0.0)) is None
        assert odometry.pose == before


class TestStartupSeedLock:
    """The failure that pinned the published pose to the origin for a whole run."""

    def test_keyframe_taken_before_odometry_is_retaken(self, floorplan):
        """Scans arrive before /odom on every launch, so this always triggered.

        A keyframe adopted with odom_pose=None leaves _ref_odom None, so
        _initial_guess can only ever return zero. Unseeded ICP down a corridor of
        parallel walls then converges to identity with near-perfect fitness, the
        reported motion never reaches the keyframe threshold, the keyframe is
        never re-taken, and _ref_odom stays None. The node published a pose stuck
        at the origin for 103 s while accepting 99% of its matches.
        """
        odometry = ScanOdometry()
        first = simulate_2d_scan(floorplan, (-8.0, 0.0, 0.0), seed=41)

        # First scan beats the first /odom message.
        assert odometry.update(first, odom_pose=None) is None
        assert odometry._ref_odom is None

        # Odometry arrives; the keyframe must be re-taken so a seed exists.
        second = simulate_2d_scan(floorplan, (-7.95, 0.0, 0.0), seed=42)
        assert odometry.update(second, odom_pose=(-7.95, 0.0, 0.0)) is None
        assert odometry._ref_odom == (-7.95, 0.0, 0.0), "keyframe not re-seeded"

        # And from here it tracks.
        truth = (-7.95, 0.0, 0.0)
        for step in range(12):
            truth = se2_compose(truth, (0.08, 0.0, 0.0))
            odometry.update(simulate_2d_scan(floorplan, truth, seed=50 + step), odom_pose=truth)

        travelled = math.hypot(odometry.pose[0] + 7.95, odometry.pose[1])
        assert travelled > 0.5, "pose still pinned near the origin: %.3f m" % travelled

    def test_a_stale_keyframe_cannot_persist_indefinitely(self, floorplan):
        """Age backstop: no failure may freeze the reference forever."""
        odometry = ScanOdometry(max_keyframe_age=5)
        truth = (-8.0, 0.0, 0.0)
        odometry.update(simulate_2d_scan(floorplan, truth, seed=61), odom_pose=truth)

        for step in range(12):
            truth = se2_compose(truth, (0.01, 0.0, 0.0))  # too small to trigger distance
            odometry.update(simulate_2d_scan(floorplan, truth, seed=70 + step), odom_pose=truth)

        assert odometry.keyframes_forced > 0, "the age backstop never fired"


class _FakeMatch:
    """Minimal stand-in for MatchResult, for exercising the gates directly."""

    def __init__(self, converged: bool, fitness: float, residual: float) -> None:
        self.converged = converged
        self.fitness = fitness
        self.mean_residual = residual
