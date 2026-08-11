# Parameter rationale

Untuned Nav2 defaults are obvious to anyone who has used it. This document says
why each value is what it is. Where a number is derived, the derivation is here;
where it is a judgement call, the trade is named.

Every constraint below is asserted by `tests/test_config_consistency.py`, so a
change that breaks one fails the test suite rather than the robot.

## The numbers everything else descends from

| Quantity | Value | Source |
|---|---|---|
| Footprint | 0.70 × 0.50 m | `amr_description/urdf/amr.urdf.xacro` |
| Inscribed radius | 0.25 m | half-width |
| Circumscribed radius | 0.43 m | √(0.35² + 0.25²) |
| Wheel radius | 0.10 m | URDF |
| Wheel separation | 0.50 m | URDF |
| Max wheel acceleration | 10 rad/s² | Gazebo diff drive plugin |
| Aisle clear width | 2.80 m | racking faces at y = ±1.40 |
| Racking bay pitch | 2.70 m | repeats for ~27 m |
| Map resolution | 0.05 m | Cartographer submap resolution |
| 2D LiDAR | 720 beams, 0.5°, 25 m, σ = 0.01 m | URDF |
| 3D LiDAR | 16 rings × 1800, ±15°, 40 m, σ = 0.01 m | URDF |

Two facts drive almost everything: the aisle is narrow relative to the robot
(1.15 m clearance either side), and the racking repeats.

---

## Costmaps

### `inflation_radius: 0.65` (both costmaps)

= circumscribed radius (0.43) + 0.22 m margin.

Worked through for this aisle: racking faces sit at |y| = 1.40, so the inflated
bands reach |y| = 0.75 and leave a **1.50 m zero-cost corridor** down the middle.
The robot is 0.50 m wide, so it can sit entirely in zero-cost cells with 0.50 m
clear either side.

Push the radius to 0.9 m and that corridor closes completely. Every plan then
runs through non-zero cost, at which point the cost gradient stops meaning
"avoid this" and starts meaning nothing at all — every cell is bad, so the
relative ordering carries no information about which side of the aisle to prefer.

Below the circumscribed radius (0.43 m) the robot's own corners can reach cells
the inflation says are safe.

### `cost_scaling_factor: 3.0`

Cost decays as `253 · exp(−k · (d − r_inscribed))`. At k = 3.0 the residual cost
at the inflation boundary is `253 · exp(−3.0 · 0.40) = 76`, so there is a step
from 76 to 0 at 0.65 m.

That discontinuity is a known property of Nav2's inflation layer and is
**accepted here rather than eliminated**. Removing it would need k ≈ 8, which
compresses the entire useful gradient into the first 10 cm beyond the robot's
own edge — the controller would then get no steering signal at all across the
1.5 m of free corridor where it actually operates. The step falls in free space
0.75 m from the racking, where `PathDist` and `GoalDist` dominate the DWB score
anyway.

### Both costmaps use identical inflation

Differing inflation between global and local is a miserable and common failure:
the global planner produces a path through a gap the local costmap considers
costly, the controller refuses to follow it, and the robot oscillates between
replanning and recovery with nothing obviously wrong in either log.

### `footprint` as a polygon, not `robot_radius`

A 0.43 m circumscribed circle around a 0.70 × 0.50 m robot claims 0.18 m of
clearance beside the robot that does not exist. In a 2.8 m aisle with 0.65 m
inflation, that is the difference between a viable plan and none.

### Local costmap `6 × 6 m`

Must exceed twice the DWB rollout distance (1.36 m) plus the inflation radius,
or trajectories run off the edge and get scored against unknown space. 6 m gives
3 m in every direction and spans the full aisle width plus both rack faces.

### `update_frequency: 5.0` (local)

The controller runs at 20 Hz but the sensors update at 10 Hz, so updating faster
than 10 Hz cannot produce new information. 5 Hz halves the cost again and is
still four costmap updates inside the robot's stopping distance at cruise.

### Two observation sources

Neither is sufficient alone:

* `scan` — the 2D LiDAR at 0.35 m. Sees pallet feet, uprights and people. Blind
  to anything overhanging above or below that one plane: a shelf beam
  protruding into the aisle at 1.6 m, or a pallet lip at 0.15 m under a rack.
* `segmentation` — non-ground returns from the 3D LiDAR, already ground-filtered.
  Covers the full column, which is what catches the overhangs.

Feeding the *raw* 3D cloud instead would put the floor into the costmap as a
wall of lethal cells.

**`segmentation` has `clearing: false`.** That cloud has had the ground removed,
so it contains no free-space evidence. Ray-tracing through it would clear cells
the sensor never saw as empty.

### `raytrace_max_range` (15 m) > `obstacle_max_range` (12 m)

If they are equal, a cell marked at the sensor's limit can never be cleared, and
the costmap accumulates permanent phantom obstacles at the edge of sensor range.

### `track_unknown_space: false`, `allow_unknown: false`

The warehouse is fully mapped. Unknown space in the global costmap therefore
means a mapping gap or a localisation failure, and planning confidently through
either is exactly wrong.

---

## DWB

### `sim_time: 1.7`

1.7 s at 0.80 m/s is a **1.36 m rollout**. This is the most consequential DWB
parameter in a narrow aisle and it is a genuine trade:

* **Longer** (2.5 s, 2.0 m): the robot sees the far side of a gap and commits
  earlier — but every trajectory that clips an obstacle anywhere in 2 m is
  scored down, so it becomes reluctant to enter a 1.2 m gap between a pallet and
  the racking at all.
* **Shorter** (1.0 s, 0.8 m): squeezes through gaps happily, but does not see
  far enough ahead to start decelerating for the aisle end, so it arrives at the
  turn too fast and overshoots.

1.7 s is just over the 0.32 m stopping distance plus the robot's own length,
which makes it the shortest horizon that guarantees every simulated trajectory
can be brought to a halt within itself. That property is asserted in the tests.

### `acc_lim_x: 1.0`, `acc_lim_theta: 2.0`

`acc_lim_x` corresponds **exactly** to `max_wheel_acceleration: 10 rad/s²` at the
0.10 m wheel radius. A planner that assumes more deceleration than the base
delivers overshoots every goal and clips every corner — and the symptom looks
like a tuning problem in the goal checker, not a mismatch with the base.

The velocity smoother carries the same limits, asserted by the tests.

### `vx_samples: 20`, `vtheta_samples: 40`, `vy_samples: 1`

800 trajectories per cycle at 20 Hz. `vy_samples` is 1 rather than 0 because the
sampler requires at least one, and `max_vel_y` is zero so it only ever produces
the single zero-lateral-velocity value — the base is differential drive.

### `linear_granularity: 0.05`

Matches the costmap resolution, so no trajectory step can skip over a lethal
cell.

### Critic weights

Scores are summed, so only ratios matter. The intent is "follow the planned
centreline closely, and treat obstacle proximity as a strong but not absolute
preference".

| Critic | Value | Default | Why |
|---|---|---|---|
| `PathDist` | 44.0 | 32.0 | In a 2.8 m aisle the centreline path is very nearly the only safe route. Letting the controller wander off it to shave a corner puts the robot in the inflation band beside the racking. |
| `PathAlign` | 40.0 | 32.0 | A differential base that drifts in heading in a corridor ends up crabbing towards a rack face. |
| `GoalDist` | 24.0 | 24.0 | Default. Raising it relative to the path critics makes the robot cut across the aisle towards the goal instead of following the corridor. |
| `GoalAlign` | 24.0 | 24.0 | Default, same reason. |
| `BaseObstacle` | 0.02 | 0.02 | Looks tiny and is not: this critic returns raw costmap cost, which runs to 253, while the path critics return distances in cells. At 0.02 an inflated cell near the inscribed radius contributes about 5 points — comparable to being one cell off the path, which is the intended balance. |

`PathAlign.forward_point_distance: 0.325` is just under half the robot length,
which keeps the alignment critic from reacting to path curvature the robot is
already inside.

`BaseObstacle.sum_scores: false` scores against the whole footprint rather than
a centre point. For a 0.70 × 0.50 m rectangle in a 2.8 m aisle the difference is
the corners, and the corners are what hit the racking.

### `xy_goal_tolerance: 0.20`, `yaw_goal_tolerance: 0.15`

The pick positions have real tolerance. Demanding centimetre accuracy from a
differential base makes it hunt around the goal without improving anything
downstream.

### `failure_tolerance: 3.0`, `required_movement_radius: 0.5` over 10 s

Both deliberately loose. A tight progress check fires while the robot is
legitimately creeping past a pallet in a narrow aisle, and a spurious recovery in
that situation makes things worse, not better.

---

## AMCL

**The most important thing in this file:** AMCL's odometry input is the *EKF
output*, not raw wheel odometry. That changes what the alpha motion-noise
parameters should be, because they describe the noise of the source AMCL is
given — and the EKF's output has already had the IMU and the scan matcher
applied to it. Leaving the alphas at values tuned for raw encoders makes the
particle cloud far more diffuse than the evidence warrants, which shows up as a
pose that wanders within a spread it should not have.

| Parameter | Value | Why |
|---|---|---|
| `alpha1`–`alpha5` | 0.05 | Low, because the input is the fused estimate. `alpha5` (strafe) is nominally zero for a non-holonomic base but left small and non-zero: a robot pushed sideways by a pallet truck does exactly this, and zero means the filter cannot represent it at all. |
| `z_hit` / `z_rand` | 0.5 / 0.5 | Deliberately conservative. A large fraction of what the LiDAR sees — pallets in the aisle, people, a parked forklift — genuinely is not in the map. Pushing `z_hit` to 0.9, common for tidy corridors, makes the filter over-penalise particles for observing real but unmapped objects. |
| `sigma_hit` | 0.2 | ~4× the map resolution. Wide enough to tolerate grid discretisation, narrow enough to still discriminate between adjacent racking bays. |
| `max_beams` | 120 of 720 | Adjacent beams in a 720-beam scan usually hit the same surface, so using all of them inflates confidence without adding information and costs 6× the compute. 120 beams still resolves a 0.10 m upright at 11 m. |
| `laser_max_range` | 20 m (sensor: 25 m) | Long returns down a 30 m aisle are the *least* informative beams: they hit the far wall, which every particle along the aisle agrees about, and they are the most affected by small yaw errors. Capping spends the beam budget on nearer structure that discriminates. |
| `update_min_d/a` | 0.20 / 0.20 | Updating a stationary robot repeatedly resamples the same measurement and artificially collapses the cloud, producing a confident pose with no new evidence behind it. |
| `recovery_alpha_slow/fast` | 0.001 / 0.10 | **Enabled deliberately.** Bays at a fixed 2.7 m pitch mean a filter locked onto the wrong bay sees perfectly good likelihood scores and will never recover unaided. The cost is that a transient — a forklift briefly filling the scan — can trigger unnecessary injection, so `alpha_fast` is at the low end. |
| `transform_tolerance` | 1.0 s | Must exceed the interval between AMCL updates, which `update_min_d` can stretch when the robot is moving slowly. |

---

## EKF

Three sources, each failing in a way the others do not:

| Source | Good at | Fails when |
|---|---|---|
| Wheel odometry | short-term velocity | a wheel slips — reports motion that did not happen |
| IMU | fast rotation, immune to slip | yaw integrates into unbounded drift |
| Scan odometry | immune to slip and drift-free over a keyframe | featureless stretches; forklift fills the view |

**Wheel odometry is fused as velocity only (`vx`, `vyaw`), never absolute pose.**
Fusing wheel x/y/yaw feeds the filter an estimate that has already integrated all
of its own past error, which double-counts and makes the covariance optimistic.

**IMU absolute yaw is deliberately NOT fused.** Two reasons. A gyro without a
heading reference has no absolute yaw to give — what it reports is its own
integrated estimate, which is already what the filter is computing. And the
obvious reference, a magnetometer, is useless in a building framed in steel.

There is a third reason specific to simulation, and it is the important one:
Gazebo's IMU plugin will happily publish a near-perfect absolute orientation
derived from simulator ground truth. Fusing it would make the numbers in
[localisation-accuracy.md](localisation-accuracy.md) far better than anything
achievable on hardware, which would make them worthless.

**Scan odometry is fused with `odom1_differential: true`.** The correct treatment
for a dead-reckoned source whose absolute value drifts but whose increments are
good. Fusing its absolute pose non-differentially would put a second competing
absolute position estimate into a filter that already has AMCL for that job.

`two_d_mode: true` — the floor is flat and the base is differential drive. This
fixes z, roll and pitch at zero and removes six unobservable states that would
otherwise absorb noise.

Process noise diagonals sit roughly an order of magnitude below the measurement
noise the sources report, which makes the filter mildly prediction-trusting
between updates — the right bias for a 30 Hz filter reading 10 Hz sensors.
Off-diagonals are zero: there is no principled way to set 105 cross-terms by
hand, and a wrong one is worse than none.

---

## Cartographer

The dominant difficulty is that bays repeat at 2.7 m over a 30 m aisle. Loop
closure will confidently match bay *n* against bay *n+1* and fold the map in on
itself, producing a warehouse a whole bay shorter than the real one. Most
non-default values exist to make that specific failure harder.

| Parameter | Value | Default | Why |
|---|---|---|---|
| `POSE_GRAPH.constraint_builder.min_score` | 0.68 | 0.55 | A match against the wrong bay scores respectably. The cost of a high threshold is fewer loop closures and more residual drift; the cost of a low one is a map folded a bay short, which is not recoverable. |
| `global_localization_min_score` | 0.72 | 0.6 | Same reasoning, applied harder. |
| `constraint_builder.sampling_ratio` | 0.30 | 0.3 | Fewer chances to form a wrong constraint. |
| `fast_correlative.linear_search_window` | 7.0 m | 7.0 | Enough to close a loop at the aisle end without reaching a similar-looking bay two along. |
| `real_time_correlative.linear_search_window` | 0.15 m | 0.1 | Deliberately far smaller than half the 2.7 m bay pitch. The window only has to cover inter-scan motion plus odometry error; a window that reached the next bay would let the search find it. |
| `use_online_correlative_scan_matching` | true | false | Expensive, and enabled anyway: the exhaustive bounded search is what stops the Ceres matcher sliding into the adjacent bay's local minimum. |
| `ceres.rotation_weight` | 60 | 40 | Yaw is the axis this environment punishes. |
| `submaps.num_range_data` | 90 | 90 | At 10 Hz and 0.6 m/s this is ~5 m of aisle, about two bays. |
| `TRAJECTORY_BUILDER_2D.max_range` | 20 m | 30 | Long returns contribute little to the local match and, being sensitive to small yaw errors, actively degrade it. |

A note on a tempting wrong move: when a map comes out smeared, the first instinct
is to raise `ceres_scan_matcher.translation_weight` relative to
`occupied_space_weight`. That makes the result track *odometry* through a genuine
slip, which is worse.

---

## Perception

| Parameter | Value | Why |
|---|---|---|
| `ground_distance_threshold` | 0.06 m | 6σ on the LiDAR's 0.01 m range noise plus margin for floor flatness. Below ~0.04 m the outer rings start being classified as obstacles, because grazing incidence inflates their range error. |
| `ground_max_tilt_deg` | 10° | The floor is flat; this covers sensor pitch under acceleration only. |
| `ground_score_subsample` | 4000 | Scoring every hypothesis against all ~28k returns is where naive RANSAC spends essentially all its time, and buys nothing: inlier ratio is estimated to well under a percent from a few thousand samples. The winner is always re-evaluated against the full cloud, so output is exact. Measured effect: 74 ms → 13.5 ms with a bit-identical plane. |
| `max_obstacle_height` | 3.0 m | Roof steel, lighting gantry and sprinkler pipework sit above. The tallest thing to avoid is a 2.2 m upright. |
| `cluster_tolerance` | 0.35 m | Sits between two hard limits. Uprights are 0.9 m apart within a bay, so below ~0.5 m avoids merging them. Ring spacing on a vertical surface is `0.035 × range`, so a fixed 0.35 m holds objects together only out to ~10 m. |
| `cluster_voxel_size` | 0.12 m | The binding constraint is that the voxel *diagonal* (0.21 m) stays below the tolerance. Finer is not free: radius-graph edge count grows with roughly the sixth power of `tolerance / voxel_size`, and 0.08 m costs an order of magnitude more time for identical output. |
| `cluster_min_points` | 25 | Roughly the return count of a 0.3 m object at the 15 m design range. |
| `range_gain` | 1.3, capped at 0.80 m | The adaptive radius. See below. |
| `enter_frames` / `exit_frames` | 2 / 5 | At 10 Hz: a zone latches within 200 ms of someone stepping in, and stays latched 500 ms after the last return, covering the worst occlusion gap behind an upright. Asymmetric on purpose — quick to notice a person, slow to forget them. |
| `max_position_variance` | 0.25 m² | 0.5 m standard deviation, about the margin the zone polygons are drawn with. Past this the zones cannot be placed accurately enough for their states to be worth publishing as valid. |

### The range-adaptive clustering radius

A 16-ring sensor spanning 30° has 2° between rings, so on a vertical surface the
rings are `0.035 × range` apart: 0.17 m at 5 m, but **0.42 m at 12 m**. Any fixed
tolerance is therefore a range limit in disguise. At 0.35 m, objects past about
10 m fall apart into one cluster per ring, each too small to survive
`min_points`, and **a person at the far end of the aisle is detected as nothing
at all**.

Raising the fixed tolerance to cover 15 m instead merges a person standing near a
rack face into the racking, which is worse.

The fix is to merge components whose bounding boxes are within
`1.3 × 0.0349 × range`, capped at 0.80 m. The cap matters: without it the radius
keeps growing and eventually exceeds the 1.1 m clearance between the aisle
centreline and a rack face, at which point a distant person merges into the
racking — precisely the outcome the adaptation exists to avoid. At 0.80 m the
radius stops growing at 17.6 m.

Measured effect on people recovered: **62.5% → 93.8%**, at a cost of ~5 ms per
scan and more structure merging (see [segmentation.md](segmentation.md)).

The tolerance for a pair uses the *nearer* of the two components, so a near
object cannot reach out and absorb a far one.
