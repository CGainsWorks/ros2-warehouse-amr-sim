# Localisation accuracy

> **Simulator.** The simulated figures on this page were measured on **Gazebo
> Classic 11 with ROS 2 Humble**, before the stack moved to gz-sim Harmonic on
> ROS 2 Jazzy in October 2026 ([simulation.md](simulation.md)). They are kept
> because the diagnosis they record (the decoupling, the z_rand fix, the bugs
> the harness found) still explains the configuration. They are not
> reproducible from this tree. The current simulated figures are in the
> README, rendered from `results/reference/gazebo_localisation.json`, and the
> offline benchmark figures on this page are unaffected by the simulator.

## What has been measured, and what has not

This distinction runs through the whole document and is not hedging — the two
categories have very different evidential weight.

| | Status | Reproduce with |
|---|---|---|
| **AMCL vs ground truth, Nav2 out of the loop** | Measured. **0.077 m, consistent.** | `make openloop` |
| **EKF vs ground truth, Nav2 out of the loop** | Measured. **0.183 m, consistent.** | `make openloop` |
| **LiDAR odometry, offline benchmark** | Measured. Good — and contradicted in simulation. | `make scan-odometry` |
| **Scan matcher in simulation** | Measured. Does not track; **disabled by default.** | `make openloop` |
| **AMCL during a Nav2 mission** | Measured. Median 0.31 m; mission does not complete. | `make eval` |
| **Recovery matrix** | Not produced — no mission completes. | `make faults` |

The offline benchmarks need only Python, numpy and scipy and are exercised by CI
on every push. The rest require a Gazebo run.

Read the offline and simulated sections together, because **they disagree**, and
the disagreement is the most useful thing on this page: the offline benchmark
credits the scan matcher with a 63% ATE reduction, and in simulation the same
code tracks nothing at all. There are no placeholder figures here — where
something has not been produced, it says so.

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
| ATE RMSE | 0.574 m | **0.215 m** | −63% |
| ATE 95th percentile | 0.851 m | 0.385 m | |
| Yaw RMSE | 0.59° | 1.04° | |
| RPE translation RMSE (1 s) | 0.0705 m | **0.0192 m** | −73% |

Scan matcher health over the same run: 397 matched, 3 rejected (0.8%), per-scan
latency median 3.0 ms, p95 6.0 ms against a 100 ms budget.

These figures are with `trim_ratio` at 0.95. At the original 0.80 the benchmark
scored better still (0.105 m, −82%) — and that setting made the matcher unusable
in simulation, for the reason given under the corridor-degeneracy bug below. A
component benchmark preferring the setting that breaks the system is exactly the
hazard this page is about.

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
| Trimmed mean residual, max | < 0.040 m |

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

12.4 m of travel, ~59 s, forward/reverse/in-place rotation/arcs, Nav2 out of
the loop:

| Source | ATE RMSE | ATE median | Yaw RMSE | NEES |
|---|---|---|---|---|
| Wheel odometry | 0.482 m | 0.020 m | 24.4° | — |
| EKF fused | 0.183 m | 0.012 m | 4.63° | 0.03, consistent |
| **AMCL** | **0.077 m** | **0.072 m** | **1.18°** | **0.73, consistent** |

AMCL began this session at 6.65 m and the EKF at 6.72 m. Both are now
statistically consistent, and AMCL is the best estimator in the stack — which is
what it is supposed to be, since correcting the EKF's drift is its whole job.

### The fix that mattered most: z_rand

`z_hit: 0.5, z_rand: 0.5` was the single largest defect in the localisation
stack, and the reasoning that produced it sounded careful. A warehouse LiDAR
sees a great deal that the static map cannot explain — pallets in the aisle,
people, a parked forklift — so half the measurement weight was given to the
uniform random term to tolerate unexplained returns.

That is a misreading of what `z_rand` does. It is **uniform**: it awards the
same likelihood to every hypothesis, however badly it fits. At 0.5 it puts a
floor of half the total weight under a pose that explains nothing at all, which
compresses the gap between a correct hypothesis and a wrong one. In a map whose
racking repeats every 2.7 m, that gap is the only thing distinguishing the right
bay from its neighbours.

The failure it produced was precise and repeatable. AMCL tracked x to within
0.01 m through 20 s of straight running, then at the instant the first in-place
rotation began it slid laterally from y = 0.00 to y = 1.47 — onto the rack face
— and stayed there, tracking the rotation rate perfectly from the wrong place.
Not drift, and not a motion-model error: a single resample onto a hypothesis
that should never have been competitive.

| | z_rand 0.5 | z_rand 0.1 |
|---|---|---|
| ATE RMSE | 1.272 m | **0.077 m** |
| Yaw RMSE | 33.9° | **1.18°** |
| NEES | 134, overconfident | **0.73, consistent** |

The unmapped-object concern was real; the remedy was wrong. Subsampling beams
and letting the obstacle layer carry unmapped geometry handles it without
blunting the filter's ability to tell two poses apart.

### Two hypotheses that were wrong, and cost a run each

Recorded because the negative results are part of the evidence.

**Wide motion noise.** The rotation failure looked like a proposal distribution
too narrow for the laser to correct, so `alpha1`/`alpha2` went to 0.20,
`max_beams` to 240 and gentle recovery injection back on. ATE went from 1.32 m
to **12.47 m**. Three parameters at once, so the result only establishes that
the combination is worse — poor experimental hygiene, and the reason every
subsequent change was made one at a time.

**Particle depletion.** Holding the count fixed at 2000 instead of 500-2000
adaptive, on the theory that KLD shrinkage was discarding the correct
hypothesis. Measured 1.56 m against the 1.27 m baseline: no better, at four
times the CPU. Reverted.

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

### Resolved: it was never the controller either

The section that used to sit here blamed DWB, on the evidence that ground truth
showed the robot using all 1.125 m of its lateral clearance and therefore "not
holding the line the planner gives it". Three of the four things it proposed
looking at were wrong, and the fourth was wrong about which obstacle mattered.
Recording that is more useful than quietly deleting it.

What the next run actually showed:

* **DWB tracks a straight line to 0.16 m over 19 m.** It was never the problem.
* Raising `PathDist`/`PathAlign` to 44/40 — reasoned, not measured — made the
  return to centreline *underdamped*. Restoring 32/32 changed nothing either
  way. Both are recorded in [parameter-rationale.md](parameter-rationale.md).
* The lateral excursion did **not** coincide with the reversal.
* The binding obstacle was not the aisle pallet at (4.0, 0.35), which is offset,
  leaves 1.45 m on its south side, and was passed without hesitation every run.

The real cause was `inflation_radius: 0.65` against a person standing at
(8.0, 0.0), dead centre of the aisle. Inflated from the obstacle and from the
racking, the two bands overlapped by 0.15 m and **closed the aisle completely**.
The full derivation is in [parameter-rationale.md](parameter-rationale.md); the
value is now 0.47, pinned inside a 5 cm window between the circumscribed radius
and aisle closure.

#### The stall, and why the aggregate ATE lied

The measurement that made this legible was AMCL error *as a function of time*
rather than aggregated over the run:

| | AMCL ATE |
|---|---|
| First 53 s, before the stall | **0.058 m** RMSE, 0.101 m max |
| Whole run | 0.476 m RMSE |

Those are the same filter on the same run. The robot drove to within 8 cm of the
person, and DWB — which hard-rejects any trajectory whose centre enters the
inscribed band — had no forward trajectory left to choose, so it sat rotating on
the spot for 21 s. The wheels turned. The body did not move. Odometry reported
0.79 m that never happened, and in a corridor whose racking repeats every 2.7 m
the laser likelihood is nearly flat along the aisle, so the scan could not
contradict it. AMCL recovered to 0.05 m within seconds of the robot moving
again.

**The aggregate said "localisation failure". The time series said "one 0.8 m
excursion inside an otherwise 0.058 m track, and something trapped the robot".**
Only the second is actionable. This is the third time in this work that an
aggregate metric pointed at the wrong subsystem — the others were odometry
scored in the wrong frame, and clustering scored against the wrong ground truth.

### Measured: the complete mission

Six waypoints, 56 m of path, Nav2 in the loop, `inflation_radius: 0.47`:

| Source | ATE RMSE | max | Yaw RMSE | Path vs truth | NEES |
|---|---|---|---|---|---|
| Wheel odometry | 2.788 m | 7.243 m | 2.01° | +0% | — |
| EKF fused | 0.320 m | 0.800 m | 0.33° | +0% | consistent |
| **AMCL** | **0.119 m** | **0.226 m** | **0.79°** | +1% | 2.05, consistent |

Three things in that table are worth more than the headline number.

**Zero discontinuities above 0.5 m.** Bay aliasing is the defining hazard of
this environment and the one a 2.7 m periodic map is built to expose. Across
56 m of driving, AMCL never jumped a bay. That is the result the `z_hit`/`z_rand`
work was actually for.

**Wheel odometry's path length is within 0% of truth.** No slip anywhere. Every
earlier mission over-reported — 276%, 169%, 56%, 36% — and every one of those
was the robot pinned against something with its wheels turning, not a sensor
fault. It is a good proxy for "did the robot ever get stuck", and it is cheap.

**Wheel ATE of 2.788 m is correct behaviour, not a regression.** Dead reckoning
with a 5% scale error over 56 m does that. Correcting it is AMCL's entire job,
and the 23× gap between them is the stack working.

The EKF's NEES reads 0.00, which the script's verdict logic reports as
"consistent". It is not inconsistent, but it is over-conservative rather than
well-calibrated, and the verdict wording does not currently distinguish the two.
That is noted in [limitations.md](limitations.md).

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
