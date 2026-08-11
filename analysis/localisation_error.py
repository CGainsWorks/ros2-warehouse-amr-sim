#!/usr/bin/env python3
"""Localisation accuracy against ground truth.

Consumes two CSV pose logs — one estimate, one reference — and reports absolute
trajectory error, relative pose error and covariance consistency. In the
simulated run the reference is Gazebo's own pose, published by the p3d plugin on
``/ground_truth/pose`` and recorded by ``amr_bringup``'s pose_logger.

CSV format, written by pose_logger and expected here::

    t,x,y,yaw,cov_xx,cov_yy,cov_yawyaw

``t`` is seconds (the ROS stamp, so simulation time). Covariance columns are
optional and may be blank or zero for a ground-truth log.

WHY THREE METRICS RATHER THAN ONE

ATE alone is the number everyone quotes and on its own it is misleading. A
filter that is smoothly 15 cm off for the whole run and one that is perfect
except for a single 3 m jump between racking bays can report similar ATE, and
they are not remotely similar systems.

* **ATE** answers "how far from the truth is the reported pose". It is what
  matters for driving to a pick face.
* **RPE** over a short window answers "how good is the local motion estimate",
  which is what the controller actually consumes and what a constant offset does
  not affect.
* **NEES** answers "is the reported covariance honest". A filter that is
  accurate but claims to be ten times more accurate than it is will be trusted
  by downstream consumers — including this stack's own zone publisher, which
  gates on covariance — in situations where it should not be.

Run directly::

    python analysis/localisation_error.py \
        --estimate results/amcl_pose.csv \
        --reference results/ground_truth.csv \
        --output results/localisation_error.json
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT / "src" / "amr_common") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src" / "amr_common"))

from amr_common.transforms import wrap_angle  # noqa: E402


@dataclass
class PoseLog:
    """A time-ordered pose log."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    cov_xx: Optional[np.ndarray] = None
    cov_yy: Optional[np.ndarray] = None
    cov_yawyaw: Optional[np.ndarray] = None

    def __len__(self) -> int:
        return len(self.t)

    @property
    def xy(self) -> np.ndarray:
        return np.stack([self.x, self.y], axis=1)

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0]) if len(self.t) > 1 else 0.0

    @property
    def path_length(self) -> float:
        if len(self) < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(self.xy, axis=0), axis=1)))


def load_pose_csv(path: str) -> PoseLog:
    """Read a pose log, tolerating missing covariance columns."""
    rows: List[dict] = []
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"t", "x", "y", "yaw"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError("%s: missing required columns %s" % (path, sorted(missing)))
        for row in reader:
            rows.append(row)

    if len(rows) < 2:
        raise ValueError("%s: need at least 2 rows, found %d" % (path, len(rows)))

    def column(name: str) -> Optional[np.ndarray]:
        if name not in rows[0]:
            return None
        try:
            return np.array([float(r[name]) if r[name] not in ("", None) else 0.0 for r in rows])
        except (TypeError, ValueError):
            return None

    log = PoseLog(
        t=column("t"),
        x=column("x"),
        y=column("y"),
        yaw=column("yaw"),
        cov_xx=column("cov_xx"),
        cov_yy=column("cov_yy"),
        cov_yawyaw=column("cov_yawyaw"),
    )

    # Sort by time. A log assembled from several topics can arrive out of order,
    # and every interpolation below assumes monotonic time.
    order = np.argsort(log.t, kind="stable")
    return PoseLog(
        t=log.t[order],
        x=log.x[order],
        y=log.y[order],
        yaw=log.yaw[order],
        cov_xx=None if log.cov_xx is None else log.cov_xx[order],
        cov_yy=None if log.cov_yy is None else log.cov_yy[order],
        cov_yawyaw=None if log.cov_yawyaw is None else log.cov_yawyaw[order],
    )


def interpolate_to(reference: PoseLog, times: np.ndarray) -> PoseLog:
    """Resample ``reference`` onto ``times`` by linear interpolation.

    Yaw is unwrapped before interpolating and re-wrapped afterwards. Interpolating
    wrapped angles directly produces a spurious full-circle sweep every time the
    trajectory crosses +-pi, which in a warehouse happens on every U-turn at the
    end of an aisle and turns a good estimate into a catastrophic-looking one.
    """
    unwrapped = np.unwrap(reference.yaw)
    return PoseLog(
        t=times,
        x=np.interp(times, reference.t, reference.x),
        y=np.interp(times, reference.t, reference.y),
        yaw=wrap_angle(np.interp(times, reference.t, unwrapped)),
    )


def _overlap_mask(times: np.ndarray, reference: PoseLog, max_gap: float) -> np.ndarray:
    """Samples that lie inside the reference's time span and near a real sample.

    Extrapolating past the ends of the reference log silently invents ground
    truth, and ``np.interp`` will do exactly that by clamping. ``max_gap`` also
    drops samples that fall in a hole in the reference — for example while the
    ground-truth publisher was starved during a heavy costmap update.
    """
    inside = (times >= reference.t[0]) & (times <= reference.t[-1])
    idx = np.searchsorted(reference.t, times).clip(1, len(reference.t) - 1)
    gap = np.minimum(np.abs(times - reference.t[idx - 1]), np.abs(reference.t[idx] - times))
    return inside & (gap <= max_gap)


@dataclass
class ErrorSummary:
    """Everything the report needs, in one serialisable object."""

    n_samples: int = 0
    duration_s: float = 0.0
    path_length_m: float = 0.0

    ate_rmse_m: float = 0.0
    ate_mean_m: float = 0.0
    ate_median_m: float = 0.0
    ate_p95_m: float = 0.0
    ate_max_m: float = 0.0
    ate_std_m: float = 0.0

    yaw_rmse_deg: float = 0.0
    yaw_mean_abs_deg: float = 0.0
    yaw_p95_deg: float = 0.0
    yaw_max_deg: float = 0.0

    rpe_delta_s: float = 1.0
    rpe_trans_rmse_m: float = 0.0
    rpe_trans_p95_m: float = 0.0
    rpe_yaw_rmse_deg: float = 0.0

    nees_mean: Optional[float] = None
    nees_consistent: Optional[bool] = None

    max_jump_m: float = 0.0
    n_jumps_over_0p5m: int = 0

    extras: Dict[str, float] = field(default_factory=dict)

    def as_dict(self) -> dict:
        out = {k: v for k, v in self.__dict__.items() if k != "extras"}
        out.update(self.extras)
        return out


def absolute_trajectory_error(estimate: PoseLog, aligned_reference: PoseLog) -> Tuple[np.ndarray, np.ndarray]:
    """Per-sample position error (m) and yaw error (rad)."""
    position = np.linalg.norm(estimate.xy - aligned_reference.xy, axis=1)
    yaw = np.abs(wrap_angle(estimate.yaw - aligned_reference.yaw))
    return position, yaw


def relative_pose_error(
    estimate: PoseLog, reference: PoseLog, delta_s: float = 1.0
) -> Tuple[np.ndarray, np.ndarray]:
    """Error in the motion accumulated over each ``delta_s`` window.

    Computed in the body frame of the earlier pose, which is the standard
    definition and the one that makes the result independent of any constant
    offset in the estimate.
    """
    if len(estimate) < 2:
        return np.zeros(0), np.zeros(0)

    # Pair every sample with the first sample at least delta_s later.
    later = np.searchsorted(estimate.t, estimate.t + delta_s, side="left")
    valid = later < len(estimate)
    i = np.flatnonzero(valid)
    j = later[valid]
    if len(i) == 0:
        return np.zeros(0), np.zeros(0)

    def body_delta(log: PoseLog, a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        dx = log.x[b] - log.x[a]
        dy = log.y[b] - log.y[a]
        c, s = np.cos(log.yaw[a]), np.sin(log.yaw[a])
        return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1), wrap_angle(log.yaw[b] - log.yaw[a])

    est_xy, est_yaw = body_delta(estimate, i, j)
    ref_xy, ref_yaw = body_delta(reference, i, j)

    return np.linalg.norm(est_xy - ref_xy, axis=1), np.abs(wrap_angle(est_yaw - ref_yaw))


def normalised_estimation_error_squared(
    estimate: PoseLog, aligned_reference: PoseLog
) -> Optional[np.ndarray]:
    """Per-sample NEES over (x, y).

    NEES is ``e^T P^-1 e``. For a consistent 2-DOF filter its expected value is
    2. Substantially above 2 means the filter is overconfident — it is more wrong
    than it admits — which is the dangerous direction. Below 2 means it is
    conservative, which wastes information but is safe.

    Returns ``None`` when the estimate log carries no usable covariance.
    """
    if estimate.cov_xx is None or estimate.cov_yy is None:
        return None

    var_x = np.asarray(estimate.cov_xx, dtype=np.float64)
    var_y = np.asarray(estimate.cov_yy, dtype=np.float64)
    usable = (var_x > 1e-12) & (var_y > 1e-12)
    if not usable.any():
        return None

    ex = estimate.x[usable] - aligned_reference.x[usable]
    ey = estimate.y[usable] - aligned_reference.y[usable]
    # Diagonal approximation: the logger records only the diagonal, so the
    # cross-covariance is unavailable. This overstates NEES when x and y errors
    # are strongly correlated, which in a corridor they are; the number is
    # therefore a conservative (pessimistic) consistency check.
    return (ex * ex) / var_x[usable] + (ey * ey) / var_y[usable]


def evaluate(
    estimate: PoseLog,
    reference: PoseLog,
    rpe_delta_s: float = 1.0,
    max_gap_s: float = 0.2,
    jump_threshold_m: float = 0.5,
) -> ErrorSummary:
    """Full comparison of an estimate against a reference."""
    mask = _overlap_mask(estimate.t, reference, max_gap_s)
    if mask.sum() < 2:
        raise ValueError(
            "estimate and reference overlap in fewer than 2 samples; check that "
            "both logs come from the same run and use the same clock"
        )

    trimmed = PoseLog(
        t=estimate.t[mask],
        x=estimate.x[mask],
        y=estimate.y[mask],
        yaw=estimate.yaw[mask],
        cov_xx=None if estimate.cov_xx is None else estimate.cov_xx[mask],
        cov_yy=None if estimate.cov_yy is None else estimate.cov_yy[mask],
        cov_yawyaw=None if estimate.cov_yawyaw is None else estimate.cov_yawyaw[mask],
    )
    aligned = interpolate_to(reference, trimmed.t)

    position_error, yaw_error = absolute_trajectory_error(trimmed, aligned)
    rpe_trans, rpe_yaw = relative_pose_error(trimmed, aligned, delta_s=rpe_delta_s)
    nees = normalised_estimation_error_squared(trimmed, aligned)

    # Discontinuities in the estimate. In this warehouse a jump usually means
    # AMCL has resampled onto a different racking bay, and it is the failure
    # that matters most operationally, so it is counted explicitly rather than
    # being left to show up as a fat tail on the ATE distribution.
    jumps = np.linalg.norm(np.diff(trimmed.xy, axis=0), axis=1) if len(trimmed) > 1 else np.zeros(0)
    reference_jumps = np.linalg.norm(np.diff(aligned.xy, axis=0), axis=1) if len(aligned) > 1 else np.zeros(0)
    excess = jumps - reference_jumps

    summary = ErrorSummary(
        n_samples=int(len(trimmed)),
        duration_s=float(trimmed.duration),
        path_length_m=float(aligned.path_length),
        ate_rmse_m=float(np.sqrt(np.mean(position_error ** 2))),
        ate_mean_m=float(position_error.mean()),
        ate_median_m=float(np.median(position_error)),
        ate_p95_m=float(np.percentile(position_error, 95)),
        ate_max_m=float(position_error.max()),
        ate_std_m=float(position_error.std()),
        yaw_rmse_deg=float(np.degrees(np.sqrt(np.mean(yaw_error ** 2)))),
        yaw_mean_abs_deg=float(np.degrees(yaw_error.mean())),
        yaw_p95_deg=float(np.degrees(np.percentile(yaw_error, 95))),
        yaw_max_deg=float(np.degrees(yaw_error.max())),
        rpe_delta_s=float(rpe_delta_s),
        max_jump_m=float(excess.max()) if len(excess) else 0.0,
        n_jumps_over_0p5m=int((excess > jump_threshold_m).sum()) if len(excess) else 0,
    )

    if len(rpe_trans):
        summary.rpe_trans_rmse_m = float(np.sqrt(np.mean(rpe_trans ** 2)))
        summary.rpe_trans_p95_m = float(np.percentile(rpe_trans, 95))
        summary.rpe_yaw_rmse_deg = float(np.degrees(np.sqrt(np.mean(rpe_yaw ** 2))))

    if nees is not None and len(nees):
        summary.nees_mean = float(nees.mean())
        # The 95% interval on the mean of N chi-squared(2) samples is roughly
        # 2 +- 2*1.96/sqrt(N). Being under the upper bound is what matters:
        # over it means overconfident.
        upper = 2.0 + 2.0 * 1.96 / math.sqrt(len(nees))
        summary.nees_consistent = bool(summary.nees_mean <= upper)
        summary.extras["nees_upper_bound"] = float(upper)
        summary.extras["nees_samples"] = float(len(nees))

    return summary


def format_report(summary: ErrorSummary, title: str = "Localisation accuracy") -> str:
    """Human-readable summary, also used as the docs table source."""
    lines = [
        title,
        "=" * len(title),
        "samples            %d over %.1f s, %.1f m travelled"
        % (summary.n_samples, summary.duration_s, summary.path_length_m),
        "",
        "Absolute trajectory error (position)",
        "  RMSE             %.3f m" % summary.ate_rmse_m,
        "  mean             %.3f m" % summary.ate_mean_m,
        "  median           %.3f m" % summary.ate_median_m,
        "  95th percentile  %.3f m" % summary.ate_p95_m,
        "  max              %.3f m" % summary.ate_max_m,
        "",
        "Absolute yaw error",
        "  RMSE             %.2f deg" % summary.yaw_rmse_deg,
        "  95th percentile  %.2f deg" % summary.yaw_p95_deg,
        "  max              %.2f deg" % summary.yaw_max_deg,
        "",
        "Relative pose error over %.1f s" % summary.rpe_delta_s,
        "  translation RMSE %.4f m" % summary.rpe_trans_rmse_m,
        "  translation p95  %.4f m" % summary.rpe_trans_p95_m,
        "  yaw RMSE         %.3f deg" % summary.rpe_yaw_rmse_deg,
        "",
        "Discontinuities",
        "  largest jump     %.3f m" % summary.max_jump_m,
        "  jumps > 0.5 m    %d" % summary.n_jumps_over_0p5m,
    ]
    if summary.nees_mean is not None:
        lines += [
            "",
            "Covariance consistency (NEES, expected 2.0 for a consistent 2-DOF filter)",
            "  mean NEES        %.2f" % summary.nees_mean,
            "  verdict          %s"
            % ("consistent" if summary.nees_consistent else "OVERCONFIDENT"),
        ]
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--estimate", required=True, help="CSV pose log of the estimate under test.")
    parser.add_argument("--reference", required=True, help="CSV pose log of ground truth.")
    parser.add_argument("--output", help="Write the summary as JSON here.")
    parser.add_argument("--rpe-delta", type=float, default=1.0, help="RPE window in seconds.")
    parser.add_argument("--title", default="Localisation accuracy")
    args = parser.parse_args(argv)

    estimate = load_pose_csv(args.estimate)
    reference = load_pose_csv(args.reference)
    summary = evaluate(estimate, reference, rpe_delta_s=args.rpe_delta)

    print(format_report(summary, args.title))

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(summary.as_dict(), indent=2), encoding="utf-8")
        print("\nwrote %s" % out_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
