# Localisation accuracy

## What has been measured, and what has not

This distinction runs through the whole document and is not hedging — the two
categories have very different evidential weight.

| | Status | Reproduce with |
|---|---|---|
| **LiDAR odometry vs ground truth** | Measured. Numbers below. | `make scan-odometry` |
| **Scan matcher per-match error** | Measured. Numbers below. | `make test` |
| **AMCL vs Gazebo ground truth** | **Not yet measured.** | `make eval` |
| **EKF vs Gazebo ground truth** | **Not yet measured.** | `make eval` |

The first two need only Python, numpy and scipy, run in seconds on any machine,
and are exercised by CI on every push. The last two require a Gazebo run, which
has not been performed — the evaluation pipeline that produces them
(`pose_logger` → `analysis/localisation_error.py`) is implemented and tested, but
no numbers from it are quoted here because none have been generated. There are no
placeholder figures in this file.

## Metrics, and why three of them

ATE alone is the number everyone quotes and on its own it is misleading. A filter
that is smoothly 15 cm off for a whole run and one that is perfect except for a
single 3 m jump between racking bays can report similar ATE, and they are not
remotely similar systems.

* **ATE** — how far the reported pose is from truth. What matters for driving to
  a pick face.
* **RPE** over a 1 s window — how good the *local* motion estimate is, which is
  what the controller consumes and what a constant offset does not affect.
* **NEES** — whether the reported covariance is honest. Expected value 2.0 for a
  consistent 2-DOF filter. Above that means overconfident, which is the dangerous
  direction: this stack's own zone publisher gates on AMCL covariance.

Jumps over 0.5 m are counted separately rather than left to show up as a fat tail
on the ATE distribution, because in this warehouse a jump usually means AMCL has
resampled onto a different racking bay, and that is the failure that matters
operationally.

---

## Measured: LiDAR odometry against ground truth

`analysis/scan_odometry_benchmark.py` drives a virtual robot 24.2 m down the
aisle at 0.6 m/s with a gentle weave, generates the 2D scans it would see, runs
the **shipped** `ScanOdometry` class over them, and compares against the exact
trajectory using the same metric code the Gazebo evaluation uses.

Wheel odometry is simulated alongside as the baseline, with an explicit error
model: 2% scale error (a 2 mm error on a 100 mm wheel radius), 1% track-width
error, per-sample Gaussian noise, and two discrete slip events of 0.25 m and
0.35 m. Slip is the interesting one — it is exactly the failure scan matching is
in the stack to cover, so a comparison without it would understate the case.

401 samples over 40 s, 24.2 m travelled:

| Metric | Wheel odometry | LiDAR odometry | Change |
|---|---|---|---|
| ATE RMSE | 0.574 m | **0.105 m** | −82% |
| ATE median | 0.380 m | 0.113 m | |
| ATE 95th percentile | 0.851 m | 0.136 m | |
| ATE max | 0.860 m | 0.145 m | |
| Yaw RMSE | 0.59° | **0.12°** | −80% |
| Yaw max | 1.24° | 0.63° | |
| RPE translation RMSE (1 s) | 0.0705 m | **0.0174 m** | −75% |
| RPE yaw RMSE (1 s) | 0.189° | 0.134° | |

Scan matcher health over the same run: 398 matched, 2 rejected (0.5%), per-scan
latency median 3.81 ms, p95 8.38 ms, max 18.6 ms against a 100 ms budget.

Note the shape of the wheel-odometry error: its *RPE* is respectable, because
locally the encoders are good. What ruins it is the systematic scale error and
the two slips, which accumulate and never wash out. That is the specific problem
the LiDAR covers.

These figures reproduce to the quoted precision on both Windows (Python 3.9,
numpy 1.26, scipy 1.13) and WSL Ubuntu 22.04 (Python 3.10, numpy 1.21,
scipy 1.8): identical ATE, identical reject count, differing only in latency
(3.1 ms median on WSL against 3.8 ms). Every stochastic step — the RANSAC seed,
the synthetic scan noise, the wheel-odometry error model — takes a fixed seed,
which is deliberate: segmentation output feeds a safety-adjacent zone publisher,
and a non-deterministic front end makes any downstream regression impossible to
bisect.

### Per-match accuracy

From `tests/test_scan_matcher.py`, over 30 matched pairs at random poses along
the aisle:

| | Value |
|---|---|
| Translation error, median | < 0.02 m |
| Translation error, 90th percentile | < 0.06 m |
| Rotation error, median | < 0.6° |
| Fitness, 5th percentile | > 0.60 |
| Trimmed mean residual, max | < 0.030 m |

The residual floor of ~0.018 m is set by the **0.5° beam spacing**, not by range
noise: at 10 m one beam step is 87 mm, so nearest-neighbour distance is dominated
by angular sampling. This is worth knowing before trying to "improve" the
matcher by reducing sensor noise.

---

## A bug this benchmark found

Worth recording, because it is the reason the benchmark exists rather than a
plausibility argument in its place.

The first working version of `ScanOdometry` produced an ATE of **5.76 m and a
30° yaw error** while reporting healthy fitness on almost every frame. The
signature was distinctive: RPE was *better* than wheel odometry (0.035 m vs
0.071 m) while ATE was twenty times worse, and there were no jumps. Good local
motion plus a large constant offset means a frame or composition error, not a
matching error.

Tracing it frame by frame showed the estimate was fine until t = 8.3 s and then
acquired a 35° yaw error over three frames. At those three frames the quality
signals *had* degraded — fitness fell from 0.80 to 0.579, the trimmed residual
rose tenfold to 0.053 m, and ICP hit its iteration cap without converging. But:

1. `ScanOdometry` computed `converged` and never looked at it.
2. `min_fitness` was 0.55, below the degraded value of 0.579.

So the bad match was accepted, a keyframe was taken at the corrupted pose, and
every subsequent match was consistent with the wrong keyframe — which is why
fitness returned to 0.80 and stayed there while the pose was permanently 35° out.

The trigger was the simulated wheel slip at t = 8.0 s poisoning the ICP seed:
the very failure the scan matcher exists to catch.

Three changes followed, all of them in the shipped code:

* **`converged` is now checked.** Regression test:
  `test_non_converged_match_is_rejected`.
* **Fitness was redefined.** It had been the fraction of points finding *any*
  correspondence, which trimming caps at `trim_ratio` by construction — in a
  dense scan almost every point finds some neighbour within 0.5 m no matter how
  wrong the transform is. It is now the fraction within a tight
  `inlier_distance` of 0.05 m, which actually discriminates.
* **A retry-unseeded path was added.** When a seeded match fails the gates, ICP
  is re-run from zero before giving up. Regression test:
  `test_retry_unseeded_recovers_from_a_poisoned_seed`.

Thresholds were then set from *measured* distributions rather than guessed. The
first attempt at 0.70 fitness rejected a correct 0.10 m match scoring 0.671;
0.60 keeps those while staying above the median of incorrect matches (0.47).

---

## Not yet measured: AMCL and the EKF against Gazebo

To generate these:

```bash
make docker-build
docker compose -f docker/docker-compose.yml run --rm eval
```

or, in a sourced workspace, `make eval`. That runs the six-waypoint mission with
`pose_logger` recording `/ground_truth/pose`, `/amcl_pose`, `/odometry/filtered`
and `/odom`, then reports ATE, RPE and NEES for each against ground truth.

`make eval` writes `results/localisation_error.json`. Paste the numbers into this
section when you have them.

### The ablation worth running

`localisation.launch.py` takes `use_scan_odometry:=false`, which leaves the EKF
on wheel odometry and IMU alone. Running the evaluation both ways isolates what
the scan matcher contributes end-to-end, as opposed to in the component
benchmark above.

### The integrity check

`/ground_truth/pose` is published by Gazebo's p3d plugin and is the simulator's
exact answer. `pose_logger` is the only thing permitted to subscribe to it, and
it verifies that at runtime every 5 seconds by enumerating subscribers on the
topic. If anything in localisation, navigation or perception is listening, it
logs `GROUND TRUTH LEAK` at error level and names the offending node.

This matters more than it might appear. There are at least three easy ways to
accidentally measure the simulator against itself here: subscribing to the p3d
topic somewhere in the stack, setting the Gazebo diff-drive plugin's
`odometry_source` to `WORLD` instead of `ENCODER`, and fusing the Gazebo IMU's
absolute orientation. The first is caught at runtime by the logger; the second
and third are caught at test time by `tests/test_config_consistency.py`. All
three would produce excellent numbers that mean nothing.
