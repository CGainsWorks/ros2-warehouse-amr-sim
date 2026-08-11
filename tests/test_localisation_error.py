"""Localisation error metrics.

Metrics code is easy to get subtly wrong and hard to notice, because the output
is a plausible-looking number either way. Each metric is therefore checked
against a case whose answer is known analytically.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "analysis"))

from localisation_error import (  # noqa: E402
    PoseLog,
    absolute_trajectory_error,
    evaluate,
    interpolate_to,
    load_pose_csv,
    normalised_estimation_error_squared,
    relative_pose_error,
)


def _straight_line(n=101, duration=10.0, speed=1.0, y=0.0, yaw=0.0):
    t = np.linspace(0.0, duration, n)
    return PoseLog(t=t, x=speed * t, y=np.full(n, y), yaw=np.full(n, yaw))


class TestLoad:
    def test_round_trip(self, tmp_path):
        path = tmp_path / "poses.csv"
        with open(path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t", "x", "y", "yaw", "cov_xx", "cov_yy", "cov_yawyaw"])
            writer.writerow([0.0, 1.0, 2.0, 0.5, 0.01, 0.02, 0.03])
            writer.writerow([0.1, 1.1, 2.1, 0.6, 0.01, 0.02, 0.03])
        log = load_pose_csv(str(path))
        assert len(log) == 2
        assert log.cov_xx[0] == pytest.approx(0.01)

    def test_missing_columns_rejected(self, tmp_path):
        path = tmp_path / "bad.csv"
        path.write_text("t,x\n0,1\n1,2\n", encoding="utf-8")
        with pytest.raises(ValueError, match="missing required columns"):
            load_pose_csv(str(path))

    def test_out_of_order_rows_are_sorted(self, tmp_path):
        path = tmp_path / "unsorted.csv"
        path.write_text("t,x,y,yaw\n2,2,0,0\n0,0,0,0\n1,1,0,0\n", encoding="utf-8")
        log = load_pose_csv(str(path))
        assert list(log.t) == [0.0, 1.0, 2.0]
        assert list(log.x) == [0.0, 1.0, 2.0]

    def test_missing_covariance_is_tolerated(self, tmp_path):
        path = tmp_path / "nocov.csv"
        path.write_text("t,x,y,yaw\n0,0,0,0\n1,1,0,0\n", encoding="utf-8")
        log = load_pose_csv(str(path))
        assert log.cov_xx is None


class TestInterpolation:
    def test_linear(self):
        reference = _straight_line(n=11, duration=10.0)
        resampled = interpolate_to(reference, np.array([0.5, 5.5]))
        assert resampled.x == pytest.approx([0.5, 5.5])

    def test_yaw_wrap_does_not_produce_a_spurious_sweep(self):
        """Interpolating across +-pi must take the short way round.

        A robot turning through pi at an aisle end goes from +3.13 to -3.13.
        Naive interpolation sweeps almost a full circle the wrong way and turns
        a good estimate into an apparently catastrophic one.
        """
        reference = PoseLog(
            t=np.array([0.0, 1.0]),
            x=np.zeros(2),
            y=np.zeros(2),
            yaw=np.array([3.13, -3.13]),
        )
        mid = interpolate_to(reference, np.array([0.5]))
        assert abs(mid.yaw[0]) > 3.0, "interpolated the long way round through zero"


class TestMetrics:
    def test_zero_error_for_identical_logs(self):
        log = _straight_line()
        position, yaw = absolute_trajectory_error(log, log)
        assert np.allclose(position, 0.0)
        assert np.allclose(yaw, 0.0)

    def test_constant_offset_is_measured_exactly(self):
        truth = _straight_line()
        estimate = PoseLog(t=truth.t, x=truth.x + 0.3, y=truth.y - 0.4, yaw=truth.yaw)
        position, _ = absolute_trajectory_error(estimate, truth)
        assert np.allclose(position, 0.5)  # 3-4-5

    def test_rpe_is_blind_to_a_constant_offset(self):
        """The property that makes RPE worth reporting alongside ATE."""
        truth = _straight_line()
        estimate = PoseLog(t=truth.t, x=truth.x + 5.0, y=truth.y + 5.0, yaw=truth.yaw)
        trans, rot = relative_pose_error(estimate, truth, delta_s=1.0)
        assert len(trans) > 0
        assert np.allclose(trans, 0.0, atol=1e-9)
        assert np.allclose(rot, 0.0, atol=1e-9)

    def test_rpe_detects_a_scale_error(self):
        truth = _straight_line(speed=1.0)
        estimate = PoseLog(t=truth.t, x=truth.x * 1.1, y=truth.y, yaw=truth.yaw)
        trans, _ = relative_pose_error(estimate, truth, delta_s=1.0)
        # 10% of 1 m/s over a 1 s window.
        assert np.median(trans) == pytest.approx(0.1, abs=0.02)

    def test_yaw_error_wraps(self):
        a = PoseLog(t=np.array([0.0, 1.0]), x=np.zeros(2), y=np.zeros(2), yaw=np.array([3.13, 3.13]))
        b = PoseLog(t=np.array([0.0, 1.0]), x=np.zeros(2), y=np.zeros(2), yaw=np.array([-3.13, -3.13]))
        _, yaw = absolute_trajectory_error(a, b)
        assert np.all(yaw < 0.05), "yaw error did not wrap; got %s" % yaw


class TestNEES:
    def test_none_without_covariance(self):
        log = _straight_line()
        assert normalised_estimation_error_squared(log, log) is None

    def test_consistent_filter_averages_about_two(self):
        rng = np.random.default_rng(0)
        n = 4000
        sigma = 0.1
        truth = PoseLog(t=np.arange(n) * 0.1, x=np.zeros(n), y=np.zeros(n), yaw=np.zeros(n))
        estimate = PoseLog(
            t=truth.t,
            x=rng.normal(0, sigma, n),
            y=rng.normal(0, sigma, n),
            yaw=np.zeros(n),
            cov_xx=np.full(n, sigma ** 2),
            cov_yy=np.full(n, sigma ** 2),
        )
        nees = normalised_estimation_error_squared(estimate, truth)
        assert nees is not None
        assert nees.mean() == pytest.approx(2.0, abs=0.15)

    def test_overconfident_filter_is_flagged(self):
        rng = np.random.default_rng(1)
        n = 2000
        truth = PoseLog(t=np.arange(n) * 0.1, x=np.zeros(n), y=np.zeros(n), yaw=np.zeros(n))
        estimate = PoseLog(
            t=truth.t,
            x=rng.normal(0, 0.5, n),
            y=rng.normal(0, 0.5, n),
            yaw=np.zeros(n),
            # Claims 0.05 m accuracy while actually being 0.5 m out.
            cov_xx=np.full(n, 0.05 ** 2),
            cov_yy=np.full(n, 0.05 ** 2),
        )
        summary = evaluate(estimate, truth)
        assert summary.nees_mean > 2.0
        assert summary.nees_consistent is False


class TestEvaluate:
    def test_end_to_end_summary(self):
        truth = _straight_line(n=201, duration=20.0)
        rng = np.random.default_rng(2)
        estimate = PoseLog(
            t=truth.t,
            x=truth.x + rng.normal(0, 0.05, len(truth)),
            y=truth.y + rng.normal(0, 0.05, len(truth)),
            yaw=truth.yaw,
        )
        summary = evaluate(estimate, truth)
        assert summary.n_samples == len(truth)
        assert summary.ate_rmse_m == pytest.approx(0.05 * math.sqrt(2), abs=0.02)
        assert summary.path_length_m == pytest.approx(20.0, abs=0.1)

    def test_refuses_non_overlapping_logs(self):
        a = _straight_line()
        b = PoseLog(t=a.t + 1000.0, x=a.x, y=a.y, yaw=a.yaw)
        with pytest.raises(ValueError, match="overlap"):
            evaluate(a, b)

    def test_does_not_extrapolate_past_the_reference(self):
        """Samples outside the reference's span must be dropped, not clamped."""
        truth = _straight_line(n=51, duration=5.0)
        estimate = _straight_line(n=101, duration=10.0)
        summary = evaluate(estimate, truth)
        assert summary.n_samples < len(estimate)
        # If the tail had been clamped rather than dropped, the last samples
        # would each show metres of invented error.
        assert summary.ate_max_m < 0.01

    def test_counts_discontinuities(self):
        truth = _straight_line(n=101, duration=10.0)
        x = truth.x.copy()
        x[50:] += 2.7  # a one-bay jump
        estimate = PoseLog(t=truth.t, x=x, y=truth.y, yaw=truth.yaw)
        summary = evaluate(estimate, truth)
        assert summary.n_jumps_over_0p5m == 1
        assert summary.max_jump_m == pytest.approx(2.7, abs=0.01)
