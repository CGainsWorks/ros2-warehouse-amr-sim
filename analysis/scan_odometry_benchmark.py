#!/usr/bin/env python3
"""LiDAR odometry accuracy against ground truth, without a simulator.

Drives a virtual robot down the warehouse aisle, generates the 2D scans it would
see, runs the **real** ``ScanOdometry`` from ``amr_localisation`` over them, and
compares the result against the exact trajectory using the **real** metrics from
``analysis/localisation_error.py``. Wheel odometry with representative error is
simulated alongside as the baseline.

    python analysis/scan_odometry_benchmark.py --output results/scan_odometry.json

WHAT THIS DOES AND DOES NOT SHOW

It measures one component: the LiDAR odometry that feeds the EKF as ``odom1``,
against the aliasing-heavy geometry that makes it hard. Because everything is
exact, ground truth is exact, and the numbers are reproducible on any machine
with numpy and scipy in a few seconds.

It is **not** a measurement of the full stack. It contains no EKF, no AMCL and
no Gazebo, so it says nothing about the end-to-end ``map -> base_link`` accuracy
that ``make eval`` produces from a simulated run. The two are reported
separately in docs/localisation-accuracy.md and must not be conflated.

The wheel-odometry error model is deliberately explicit rather than tuned to
flatter the comparison:

* a systematic scale error, which is what an imperfectly known wheel radius
  produces and which no amount of averaging removes;
* a small systematic track-width error, producing yaw drift proportional to
  distance;
* per-sample Gaussian noise on both;
* discrete slip events, where the wheels report motion that did not happen.

Slip is the interesting one. It is exactly the failure that scan matching is in
the stack to cover, so a comparison run without it would understate the case for
having the scan matcher at all.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
for _pkg in ("amr_common", "amr_localisation"):
    _path = str(REPO_ROOT / "src" / _pkg)
    if _path not in sys.path:
        sys.path.insert(0, _path)
if str(REPO_ROOT / "analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "analysis"))

from amr_common.transforms import wrap_angle  # noqa: E402
from amr_localisation.core.scan_odometry import ScanOdometry  # noqa: E402
from amr_localisation.core.synthetic2d import (  # noqa: E402
    simulate_2d_scan,
    warehouse_floorplan,
)
from localisation_error import PoseLog, evaluate, format_report  # noqa: E402


@dataclass
class Trajectory:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    v: np.ndarray
    omega: np.ndarray

    def pose(self, index: int) -> Tuple[float, float, float]:
        return (float(self.x[index]), float(self.y[index]), float(self.yaw[index]))

    def to_log(self) -> PoseLog:
        return PoseLog(t=self.t, x=self.x, y=self.y, yaw=self.yaw)


def aisle_traverse(
    x_start: float = -12.0,
    x_end: float = 12.0,
    speed: float = 0.6,
    rate: float = 10.0,
    weave_amplitude: float = 0.25,
    weave_period_m: float = 9.0,
) -> Trajectory:
    """A run down the aisle with a gentle weave.

    The weave is not decoration. A perfectly straight traverse down a corridor
    is the one trajectory where a wrong yaw estimate costs almost nothing, and
    it also gives the scan matcher no rotational excitation at all. The weave
    keeps the run representative of a robot actually tracking a path around
    obstacles, at an amplitude (0.25 m over a 9 m period, so under 10 degrees of
    heading) that stays within what the aisle allows.
    """
    distance = abs(x_end - x_start)
    n = int(round(distance / speed * rate)) + 1
    t = np.arange(n) / rate

    x = np.linspace(x_start, x_end, n)
    k = 2.0 * math.pi / weave_period_m
    y = weave_amplitude * np.sin(k * (x - x_start))
    # Heading is the path tangent, so the robot is always facing the way it
    # moves — which is what a differential drive following a path does.
    yaw = np.arctan2(weave_amplitude * k * np.cos(k * (x - x_start)), 1.0)

    v = np.full(n, speed)
    omega = np.gradient(np.unwrap(yaw), t)
    return Trajectory(t=t, x=x, y=y, yaw=yaw, v=v, omega=omega)


def simulate_wheel_odometry(
    truth: Trajectory,
    scale_error: float = 1.02,
    track_error: float = 1.01,
    noise_v: float = 0.01,
    noise_omega: float = 0.01,
    slip_events: Sequence[Tuple[float, float]] = ((8.0, 0.25), (22.0, 0.35)),
    seed: int = 0,
) -> PoseLog:
    """Dead-reckon a wheel-odometry pose with representative systematic error.

    ``scale_error`` of 1.02 is a 2% overestimate of distance travelled, which is
    what a 2 mm error on a 100 mm wheel radius gives — well inside what an
    uncalibrated robot ships with. ``slip_events`` are ``(time_s, metres)``
    pairs of phantom forward motion, representing a drive wheel spinning on a
    wet patch.
    """
    rng = np.random.default_rng(seed)
    n = len(truth.t)
    dt = float(np.median(np.diff(truth.t)))

    x = np.zeros(n)
    y = np.zeros(n)
    yaw = np.zeros(n)
    x[0], y[0], yaw[0] = truth.pose(0)

    slip_by_index = {}
    for slip_time, slip_metres in slip_events:
        index = int(round(slip_time / dt))
        if 0 < index < n:
            slip_by_index[index] = slip_metres

    for i in range(1, n):
        v = truth.v[i] * scale_error + rng.normal(0.0, noise_v)
        w = truth.omega[i] * track_error + rng.normal(0.0, noise_omega)

        step = v * dt + slip_by_index.get(i, 0.0)
        yaw[i] = wrap_angle(yaw[i - 1] + w * dt)
        # Midpoint heading: integrating with the start-of-step heading biases
        # every turn outwards and would exaggerate the wheel odometry's error.
        mid = yaw[i - 1] + 0.5 * w * dt
        x[i] = x[i - 1] + step * math.cos(mid)
        y[i] = y[i - 1] + step * math.sin(mid)

    return PoseLog(t=truth.t.copy(), x=x, y=y, yaw=yaw)


def run_scan_odometry(
    truth: Trajectory,
    segments,
    use_odom_guess: bool = True,
    wheel: Optional[PoseLog] = None,
    range_sigma: float = 0.01,
    n_beams: int = 720,
    seed: int = 0,
    **kwargs,
) -> Tuple[PoseLog, dict]:
    """Run the real ScanOdometry over simulated scans of the aisle."""
    odometry = ScanOdometry(initial_pose=truth.pose(0), **kwargs)

    n = len(truth.t)
    xs, ys, yaws, ts = [], [], [], []
    latencies: List[float] = []

    for i in range(n):
        points = simulate_2d_scan(
            segments,
            truth.pose(i),
            n_beams=n_beams,
            range_sigma=range_sigma,
            seed=seed + i,
        )
        guess = None
        if use_odom_guess and wheel is not None:
            guess = (float(wheel.x[i]), float(wheel.y[i]), float(wheel.yaw[i]))

        started = time.perf_counter()
        odometry.update(points, odom_pose=guess)
        latencies.append((time.perf_counter() - started) * 1e3)

        # Record the current estimate every cycle, including cycles where the
        # match was rejected and the pose therefore did not move. That is what
        # a consumer of the topic experiences and is the honest thing to score.
        ts.append(float(truth.t[i]))
        xs.append(odometry.pose[0])
        ys.append(odometry.pose[1])
        yaws.append(odometry.pose[2])

    log = PoseLog(
        t=np.array(ts), x=np.array(xs), y=np.array(ys), yaw=np.array(yaws)
    )
    latency = np.array(latencies)
    stats = {
        "matched": odometry.matched,
        "rejected": odometry.rejected,
        "reject_rate": odometry.reject_rate,
        "latency_ms_median": float(np.median(latency)),
        "latency_ms_p95": float(np.percentile(latency, 95)),
        "latency_ms_max": float(latency.max()),
    }
    return log, stats


def write_csv(path: Path, log: PoseLog) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "x", "y", "yaw"])
        for row in zip(log.t, log.x, log.y, log.yaw):
            writer.writerow(["%.6f" % v for v in row])


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--speed", type=float, default=0.6, help="Traverse speed, m/s.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", help="Write the summary as JSON here.")
    parser.add_argument("--csv-dir", help="Also write the raw pose logs as CSV here.")
    parser.add_argument(
        "--no-odom-guess",
        action="store_true",
        help="Run ICP unseeded. Shows what the wheel-odometry prior is worth.",
    )
    args = parser.parse_args(argv)

    truth = aisle_traverse(speed=args.speed)
    segments = warehouse_floorplan()

    wheel = simulate_wheel_odometry(truth, seed=args.seed)
    scan, scan_stats = run_scan_odometry(
        truth,
        segments,
        use_odom_guess=not args.no_odom_guess,
        wheel=wheel,
        seed=args.seed,
    )

    truth_log = truth.to_log()
    wheel_summary = evaluate(wheel, truth_log, rpe_delta_s=1.0)
    scan_summary = evaluate(scan, truth_log, rpe_delta_s=1.0)

    print(format_report(wheel_summary, "Wheel odometry alone (baseline)"))
    print()
    print(format_report(scan_summary, "LiDAR scan odometry"))
    print()
    print("Scan matcher health")
    print("  matched          %d" % scan_stats["matched"])
    print("  rejected         %d (%.1f%%)" % (scan_stats["rejected"], 100.0 * scan_stats["reject_rate"]))
    print(
        "  per-scan latency median %.2f ms, p95 %.2f ms, max %.2f ms"
        % (scan_stats["latency_ms_median"], scan_stats["latency_ms_p95"], scan_stats["latency_ms_max"])
    )

    improvement = (
        1.0 - scan_summary.ate_rmse_m / wheel_summary.ate_rmse_m
        if wheel_summary.ate_rmse_m
        else 0.0
    )
    print()
    print("ATE RMSE: %.3f m wheel -> %.3f m scan (%.0f%% reduction)"
          % (wheel_summary.ate_rmse_m, scan_summary.ate_rmse_m, 100.0 * improvement))

    result = {
        "trajectory": {
            "speed_mps": args.speed,
            "duration_s": float(truth.t[-1]),
            "path_length_m": float(truth_log.path_length),
            "samples": int(len(truth.t)),
        },
        "wheel_odometry": wheel_summary.as_dict(),
        "scan_odometry": scan_summary.as_dict(),
        "scan_matcher": scan_stats,
        "ate_rmse_reduction": improvement,
        "config": {"seed": args.seed, "odom_guess": not args.no_odom_guess},
    }

    if args.csv_dir:
        csv_dir = Path(args.csv_dir)
        write_csv(csv_dir / "ground_truth.csv", truth_log)
        write_csv(csv_dir / "wheel_odometry.csv", wheel)
        write_csv(csv_dir / "scan_odometry.csv", scan)
        print("\nwrote pose logs to %s" % csv_dir)

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print("wrote %s" % out_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
