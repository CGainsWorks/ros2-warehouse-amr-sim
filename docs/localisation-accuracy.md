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

## Measured in simulation — and still not good enough

The Gazebo evaluation runs end to end. The numbers say the stack does **not** yet
localise acceptably, and they are published in full rather than withheld until
they improve.

Latest run, scan odometry enabled, 148 s and 14.5 m of ground-truth travel:

| | value |
|---|---|
| AMCL ATE RMSE | 6.65 m |
| AMCL yaw RMSE | 102.9° |
| AMCL jumps > 0.5 m | 2 |
| Wheel odometry ATE RMSE | 1.35 m |
| Mission outcome | does not complete |

### Path length is the diagnostic, not ATE

`analysis/odometry_audit.py` compares how far each source *thinks* the robot
went. It localises a fault to one link in the chain in a way ATE cannot: a
lagging estimate and a noisy one report similar ATE, but only one of them shows
up as missing distance.

| Source | Path | Net | vs truth |
|---|---|---|---|
| Ground truth | 14.54 m | 11.53 m | — |
| Wheel odometry | 19.71 m | 10.46 m | +36% |
| LiDAR scan odometry | 5.08 m | **0.10 m** | −65% |
| EKF fused | 13.39 m | 5.37 m | −8% |
| AMCL | 12.49 m | 4.30 m | −14% |

Two things fall straight out. Wheel odometry over-reports by 36%, which is slip
during the recovery spins — exactly what the LiDAR is in the stack to correct,
and it is correcting it: the EKF lands within 8% of truth where its wheel input
is 36% over. And the scan matcher's own published pose has a **net displacement
of 0.10 m** while the robot moved 11.5 m. It is damping the wheel slip usefully
and tracking nothing.

### The startup seed lock — found, fixed

The first version of this run had the EKF at −34% instead of −8%. The cause was a
race with a self-reinforcing lock:

1. Scans arrive before the first `/odom` on every launch, so the first keyframe
   was adopted with `_ref_odom = None`.
2. `_initial_guess` therefore returned zero, permanently.
3. Unseeded ICP down a corridor of parallel walls converges to *identity* with
   near-perfect fitness — the walls align at any x, and the 80% trimming
   discards the racking uprights, which are the only points that constrain
   translation along the aisle, as outliers.
4. Reported motion never reached the keyframe threshold, so the keyframe was
   never re-taken, so `_ref_odom` stayed `None`.

The node published a pose pinned to the origin for 103 s while accepting 99% of
its matches and reporting a 1% reject rate. Every health signal it had said it
was fine.

Two fixes: re-take the keyframe as soon as wheel odometry becomes available, and
a `max_keyframe_age` backstop so that no fault which suppresses measured motion
can freeze the reference indefinitely. Both have regression tests in
`TestStartupSeedLock`.

### Outstanding, and honestly outstanding

1. **The scan matcher still does not track.** Net 0.10 m over 11.5 m even with
   the seed restored. The corridor degeneracy in step 3 above is the likely
   remaining cause and the trimming is the prime suspect: it removes precisely
   the sparse points that constrain motion along a corridor. Worth testing with
   `trim_ratio: 1.0`, or with a point-to-line metric which does not have this
   failure mode.
2. **AMCL diverges** (6.65 m ATE, 102.9° yaw). Downstream of (1) in part, but
   not entirely.
3. **No mission completes.** Capped at 400 s wall clock, and Gazebo runs at
   roughly 0.3–0.5× real time here.

### Why this is worth publishing

The offline benchmark reports the scan matcher cutting ATE by 82%. In
simulation the same code, unchanged, tracked nothing at all. Both measurements
are correct and only the second is the system.

The benchmark seeds every match from accurate wheel odometry and scores the
matcher's own poses on clean 10 Hz synthetic scans. It cannot see a startup race
that never occurs in it, and it cannot see corridor degeneracy because the seed
hides it. That is the argument for end-to-end evaluation over a green component
benchmark, and it is why the ablation switch and the odometry audit exist.

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
