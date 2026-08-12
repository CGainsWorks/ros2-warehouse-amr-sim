# Parameter rationale

Untuned Nav2 defaults are obvious to anyone who has used it. This document says
why each value is what it is. Where a number is derived, the derivation is here;
where it is a judgement call, the trade is named.

Every constraint below is asserted by `tests/test_config_consistency.py`, so a
change that breaks one fails the test suite rather than the robot.

## The numbers everything else descends from

| Quantity | Value | Source |
|---|---|---|
| Chassis | 0.70 × 0.50 m | `amr_description/urdf/amr.urdf.xacro` |
| Footprint | 0.70 × 0.55 m | widest point — the wheels, not the chassis |
| Inscribed radius | 0.275 m | half-width |
| Circumscribed radius | 0.445 m | √(0.35² + 0.275²) |
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

### `inflation_radius` (both costmaps)

Set to 0.47 m. The derivation is below; the short version is that it is pinned
inside a 5 cm window by the robot's own geometry and by a person standing in the
aisle.

Worked through for this aisle: racking faces sit at |y| = 1.40, so the inflated
bands reach |y| = 0.93 and leave a **1.86 m zero-cost corridor** down the middle
of an empty aisle. The robot is 0.55 m wide, so it can sit entirely in zero-cost
cells with 0.655 m clear either side.

An empty aisle is not the case that binds, though, and sizing the radius against
it is what produced the failure described two sections down.

Push the radius to 0.9 m and that corridor closes completely. Every plan then
runs through non-zero cost, at which point the cost gradient stops meaning
"avoid this" and starts meaning nothing at all — every cell is bad, so the
relative ordering carries no information about which side of the aisle to prefer.

Below the circumscribed radius (0.43 m) the robot's own corners can reach cells
the inflation says are safe.

### `inflation_radius: 0.47` — a 5 cm feasible window

This value is squeezed between two hard bounds, and it is the number that
decided whether the mission could complete at all.

**Lower bound — the circumscribed radius, 0.445 m.** Below this the robot's own
corners reach cells the inflation calls safe.

**Upper bound — aisle closure, 0.498 m.** A person stands at (8.0, 0.0), dead
centre of the aisle, and is 0.5 m across. Inflate them by `r` and the racking by
`r` and the zero-cost lane on each side is

```
lane = (aisle_half_width - person_half_width) - 2r  =  1.15 - 2r
```

The lane has to be wide enough that position uncertainty does not push the robot
out of it. Localisation measures 0.077 m RMSE (open loop, against ground truth),
so requiring `lane >= 2 x RMSE = 0.154 m` gives `r < 0.498`.

That leaves the interval **[0.445, 0.498]**, and 0.47 is its midpoint. The lane
is then 0.21 m wide. `tests/test_config_consistency.py` asserts both bounds and
reads the obstacle positions from the world generator, so moving the person
re-derives the limit rather than silently invalidating it.

#### What 0.65 did

The previous value was 0.65, derived for an *empty* aisle, where it leaves a
comfortable 1.50 m corridor. With a person in the middle of that aisle:

```
lane = 1.15 - 1.30 = -0.15 m
```

The aisle was **closed by 15 cm**. Not tight — closed. Every route past x = 8
ran through non-zero cost, so the cost gradient carried no information about
which side to pass, and NavFn ended the run reporting
`GridBased failed to create plan with tolerance 0.50`.

The measured failure matched the arithmetic exactly. The robot drove up to
x = 7.32 — its front bumper 8 cm from the person — and sat there for **21
seconds**, rotating on the spot, because DWB hard-rejects any trajectory whose
centre enters the inscribed band and every forward trajectory did. It eventually
found its way through the north gap by chance, arrived at the far end with no
lateral room left, touched the rack face at y = 1.17, and aborted.

Two things are worth taking from that. The first is that the binding constraint
was never the aisle pallet at (4.0, 0.35): that one is offset, leaves 1.45 m on
its south side, and the robot passed it without hesitating. It was the obstacle
in the *centre* of the aisle, which is the one case where inflation from both
sides meets in the middle. The second is that the 21-second stall did real
damage to the estimator — see below.

#### The stall corrupted localisation, and that is not AMCL's fault

AMCL held **0.058 m RMSE, 0.101 m worst case** for the first 53 seconds of that
mission. Then the robot stalled against the person with its wheels still
turning, and over the next 12 seconds AMCL's estimate slid 0.79 m forward while
the robot did not move at all.

The wheels were slipping, odometry reported motion that had not happened, and in
a corridor whose racking repeats every 2.7 m the laser likelihood is nearly flat
along the aisle — so the scan could not contradict it. AMCL recovered to 0.05 m
within seconds of the robot moving again.

This is worth stating because the whole-mission ATE for that run is 0.476 m,
which looks like a localisation failure and is not one. It is one 0.8 m
excursion, caused by a costmap that had trapped the robot, inside an otherwise
0.058 m track. Reporting the aggregate alone would have sent the next day's work
to entirely the wrong place.

### `cost_scaling_factor: 5.8`


Cost decays as `253 · exp(−k · (d − r_inscribed))`. This value is **not chosen
independently** — it follows from the inflation radius. Holding the residual cost
at the inflation boundary fixed at 82 while the radius moved from 0.65 to 0.47
gives `k = ln(253/82) / (0.47 − 0.275) = 5.8`, so the step from 82 to 0 at the
boundary is exactly what it was before and the trade below is unchanged.

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

A 0.445 m circumscribed circle around a 0.70 × 0.55 m robot claims 0.17 m of
clearance beside the robot that does not exist. In a 2.8 m aisle with 0.47 m
inflation, that is the difference between a viable plan and none.

### The footprint is wider than the chassis, on purpose

0.55 m against a 0.50 m chassis. The drive wheels are 0.05 m wide centred at
y = ±0.25, so they reach y = ±0.275 — 25 mm proud of the chassis on each side.

Setting the footprint from `base_width`, which is the obvious thing to do and
what this repository did until the robot was actually driven, understates the
robot by exactly the parts that stick out furthest. Those parts are the wheels,
and in a 2.8 m aisle the wheels are what would clip a racking upright. The
footprint tracks the widest point of the robot, whatever that happens to be;
`test_costmap_footprint_covers_the_widest_part_of_the_robot` computes it from
`wheel_separation` and `wheel_width` rather than trusting `base_width`.

### Local costmap `6 × 6 m`

Must exceed twice the DWB rollout distance (0.77 m) plus the inflation radius,
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

### A Nav2 params file resolves nothing

Worth its own heading because it cost a whole evaluation run.

`nav2_params.yaml` is plain YAML read by rclcpp. It performs **no substitution**.
Writing the natural-looking

```yaml
default_nav_to_pose_bt_xml: $(find-pkg-share amr_navigation)/behavior_trees/navigate_w_recovery.xml
```

passes that string through verbatim, and `bt_navigator` then fails to configure
with:

```
[bt_navigator] Couldn't open input XML file: $
[lifecycle_manager_navigation] Failed to change state for node: bt_navigator
[lifecycle_manager_navigation] Failed to bring up all requested nodes. Aborting bringup.
```

The consequence is disproportionate: the lifecycle manager aborts the **entire**
navigation bringup, so every subsequent goal is rejected in under a second, and
the mission log says only `mission goal rejected by Nav2` with no hint as to
why. Three lines in the simulation log explain it; nothing in the mission output
does.

Anything needing a package-relative path belongs in the launch file, which can
resolve `FindPackageShare`. `navigation.launch.py` sets both tree paths by
appending a parameter dictionary *after* the params file, so the override wins.
`test_params_file_contains_no_launch_substitutions` fails the build if the
pattern reappears.

## DWB

### `sim_time: 1.7`

1.7 s at 0.45 m/s is a **0.77 m rollout**. This is the most consequential DWB
parameter in a narrow aisle and it is a genuine trade:

* **Longer** (2.5 s, 1.1 m): the robot sees the far side of a gap and commits
  earlier — but every trajectory that clips an obstacle anywhere in 1.1 m is
  scored down, so it becomes reluctant to enter the 0.21 m lane past a person at
  all. The rollout must be shorter than the gap it is threading, or the
  controller is committing to distance it cannot see the end of.
* **Shorter** (1.0 s, 0.45 m): squeezes through gaps happily, but does not see
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
| `PathDist` | 32.0 | 32.0 | Default, **restored after the raised value was measured**. See below. |
| `PathAlign` | 32.0 | 32.0 | Default, restored for the same reason. |
| `GoalDist` | 24.0 | 24.0 | Default. Raising it relative to the path critics makes the robot cut across the aisle towards the goal instead of following the corridor. |
| `GoalAlign` | 24.0 | 24.0 | Default, same reason. |
| `BaseObstacle` | 0.02 | 0.02 | Looks tiny and is not: this critic returns raw costmap cost, which runs to 253, while the path critics return distances in cells. At 0.02 an inflated cell near the inscribed radius contributes about 5 points — comparable to being one cell off the path, which is the intended balance. |

#### The path gains were raised by argument and lowered by measurement

They were 44 and 40, reasoned from "in a 2.8 m aisle the centreline is very
nearly the only safe route, so track it hard". That produced an underdamped
controller: after the legitimate excursion around the aisle pallet the robot
overshot the return to the centreline, reaching y = +0.98 with nothing on that
side to avoid, and stopped in the rack inflation.

A path-following gain is a gain. Raising it does not make the robot more
obedient, it makes each correction more violent relative to the distance the
controller has already committed to, and the result oscillates. Restoring the
defaults did not fix the overshoot either, which is what pointed at speed rather
than gain as the real constraint.

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
