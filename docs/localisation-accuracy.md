# Localisation accuracy

## What has been measured, and what has not

This distinction runs through the whole document and is not hedging — the two
categories have very different evidential weight.

| | Status | Reproduce with |
|---|---|---|
| **LiDAR odometry, offline benchmark** | Measured. Good. | `make scan-odometry` |
| **Scan matcher per-match error** | Measured. Good. | `make test` |
| **AMCL vs Gazebo ground truth** | Measured. **Not acceptable.** | `make eval` |
| **EKF vs Gazebo ground truth** | Measured. Within 8% of true distance. | `make eval` |
| **Recovery matrix** | Not produced — no mission completes. | `make faults` |

The first two need only Python, numpy and scipy, run in seconds on any machine,
and are exercised by CI on every push. The rest require a Gazebo run.

Read the offline and simulated sections together, because **they disagree**, and
the disagreement is the most useful thing on this page: the offline benchmark
says the scan matcher cuts ATE by 82%, while in simulation the same code tracked
nothing at all. There are no placeholder figures here — where something has not
been produced, it says so.

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

## Measured in simulation

### The evaluation had to be decoupled first

A Nav2 mission cannot be used to tune localisation, because the two are
coupled: a localisation error makes the costmap wrong, Nav2 fails to progress
and runs recovery spins, the robot clips something, the wheels slip, and the
slip corrupts the odometry localisation depends on. Every number moves for
reasons unrelated to whatever is being tuned.

The evidence that this was not hypothetical: wheel odometry "over-reported by
276%", then 169%, then 56%, then 36% across successive mission runs. In the
open-loop harness, with the same code and no Nav2, it over-reports by **0%**.
None of those numbers were sensor faults. They were the robot pinned against a
pallet or a rack face with its wheels turning.

`scripts/run_openloop.sh` drives a fixed velocity profile with Nav2 out of the
loop. Nothing reads any estimate, so the trajectory is identical run to run and
the resulting ATE measures the estimator alone. Everything below comes from it.

### Where the chain stands

12.4 m of travel, ~59 s, forward/reverse/in-place rotation/arcs:

| Source | Path vs truth | ATE RMSE | ATE median | Yaw RMSE | NEES |
|---|---|---|---|---|---|
| Wheel odometry | −0% | 0.482 m | 0.020 m | 24.4° | — |
| **EKF fused** | **−1%** | **0.157 m** | **0.012 m** | **3.91°** | **0.02, consistent** |
| AMCL | +18% | 1.272 m | 1.403 m | 33.9° | 134, overconfident |

The EKF is now the best estimator in the stack and the only one whose reported
covariance is statistically honest. It started this session at 6.72 m.

### Three bugs the harness found

**1. The plugin's wheel twist has no sign.** `gazebo_ros_diff_drive` computes
its ENCODER forward velocity as `sqrt(dx² + dy²)/dt` — a magnitude, never
negative. The pose it integrates is correctly signed; the twist it publishes is
not. The EKF fused the twist, so every reverse was integrated as a forward.

On a profile driving 3.5 m out and 3.5 m back, ground truth and the wheel pose
both returned to the start (net 1.2 m, 1.34 m) while the EKF ran monotonically
to a net 11.7 m — with a *perfectly correct path length*, because it was
integrating |v|, and a good yaw. That combination is what made it look like a
position bug rather than a sign bug. Fixed by fusing the pose differentially,
which is still velocity fusion but keeps the sign. **EKF ATE 6.72 m → 0.157 m.**

**2. Accelerometer bias integrated into position.** `ax` was fused on the
reasoning that more information is better. The simulated IMU carries a
0.1 m/s² bias by design and `robot_localization` does not estimate accelerometer
bias, so the filter integrated a constant acceleration into velocity and then
into position, unopposed. A wheel encoder measures forward velocity directly and
far better. Now yaw rate only.

**3. Trimmed ICP is degenerate in a corridor.** Measured, unseeded, against a
true 0.10 m motion along the aisle:

| trim_ratio | error | |
|---|---|---|
| 0.60 | 0.099 m | reported zero motion |
| 0.80 (shipped) | 0.045 m | lost half the motion |
| 0.95 | 0.006 m | |

`max_correspondence_distance` is already the outlier gate. Trimming beyond it
discards good correspondences with large residuals, and in a corridor of
parallel walls those are the racking uprights — the only features constraining
motion along the aisle. Raised to 0.95.

### An experiment that failed

AMCL breaks specifically during in-place rotation: under 1.5° of yaw error and
0.06 m of position error for the first 20 s, then it loses itself during the
first 180° turn and sits 60–70° out for the rest of the run.

The obvious reading is a proposal distribution too narrow for the laser to
correct, so `alpha1`/`alpha2` were raised to 0.20, `max_beams` to 240 and gentle
recovery injection re-enabled. **That made it far worse — ATE 1.32 m → 12.47 m,
yaw 51° → 131°.** A wide proposal in a corridor whose racking repeats every
2.7 m gives the filter more wrong hypotheses to find, and injected particles
latch onto them. All three were reverted.

Three parameters were changed at once, so the result only establishes that the
combination is worse, not which one did the damage. That is a methodological
failure worth naming rather than hiding.

### Outstanding

1. **AMCL's rotation failure is unresolved.** It is the weakest link in the
   chain and the only estimator worse than its own input. Next step is to change
   one parameter at a time against the open-loop harness, which now gives a
   stable number to compare against.
2. **The scan matcher does not track in the simulated corridor** (net 0.1 m over
   11.5 m) despite working on the offline benchmark. It is now **disabled by
   default**: with it fused the EKF loses 32% of the motion and scores 3.26 m,
   without it 0.157 m. Shipping a component that measurably degrades the
   estimate would be wrong; the switch remains for when it is fixed.
3. **No mission completes.** With localisation fixed in open loop, a Nav2
   mission still aborts: wheel odometry over-reports 108% in the mission and 0%
   in open loop, which means the robot is still colliding. The residual AMCL
   error is enough to put it where it does not think it is.

### Why the offline benchmark did not catch any of this

It reports the scan matcher cutting ATE by 63%, and in simulation the same code
tracks nothing. Both are correct. The benchmark seeds every match from accurate
wheel odometry and scores the matcher's own poses on clean synthetic scans; it
cannot see a startup race that never occurs in it, corridor degeneracy that the
seed hides, or a plugin's unsigned twist that it never touches.

A component benchmark measures a component. Only the system measures the system.

## How to regenerate these

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
