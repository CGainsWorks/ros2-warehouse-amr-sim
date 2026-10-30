# ros2-warehouse-amr-sim

An autonomous mobile robot navigation stack for a simulated warehouse, plus a 3D
LiDAR segmentation pipeline that produces occupancy state for defined work zones,
and a VDA 5050 fleet interface so a master control can dispatch it over MQTT.
Built on ROS 2 Jazzy and gz-sim Harmonic, demonstrating the same architecture as
an industrial AMR deployment. Every figure from the simulator is labelled as
simulated; no hardware was used.

> ### Clean-room reimplementation
> This repository is an **independent reimplementation of a problem class**, built from
> scratch on personal time using public or synthetic data and open-source simulation.
> It is **not** derived from, and contains no material belonging to, any employer or
> client: no employer source code, schematics, calibration data, client data or internal
> documentation. It exists to demonstrate the architecture and engineering approach only.

---

```
map ──────── AMCL ────────► odom ──── EKF (wheel + IMU) ──────────► base_footprint
                              ▲
   ┌──────────────────────────┴───────────────────────────┐
   │                                                       │
2D LiDAR ──► scan matcher (ICP, keyframed; wired into the  ┤
   │         EKF, off by default, see below)               │
   └──► AMCL ──► Nav2: NavFn ► DWB ► velocity smoother ► /cmd_vel
                    ▲                                      │
   VDA 5050 order ──┘ (MQTT, amr_fleet)                    │
3D LiDAR ──► RANSAC ground removal ──► Euclidean clustering ┤──► /zones
                                          │                 │    (7 work zones,
                                          └──► voxel grid ──┘     debounced booleans)
                                          └──► costmap obstacle layer
```

**A 2.8 m aisle flanked by racking that repeats every 2.7 m for 30 m.** Those two
facts drive nearly every parameter in the stack: the aisle is narrow relative to
the 0.70 × 0.50 m robot, and the periodicity means a confidently wrong pose one
bay out is always available to the localiser.

---

## Quick start

Two independent entry points. The first needs nothing but Python.

```bash
# Reproduces every segmentation and scan-matching number below. ~15 seconds.
pip install -r requirements.txt
make check
```

```bash
# The full simulation. Needs Docker.
make docker-demo

# The simulated robot behind its VDA 5050 interface, with a Mosquitto broker.
docker compose -f docker/docker-compose.yml up mosquitto fleet
```

`make check` runs the real algorithms (RANSAC ground removal, Euclidean
clustering, ICP scan matching, the error metrics) against a ray-traced warehouse
scene, with no ROS and no simulator. CI runs it on every push, and alongside it
builds the workspace on Jazzy and runs two headless integration tests against
gz-sim (see [CI](#what-ci-runs)).

---

## Results

Every number here is produced by a script in this repository. Nothing is
estimated or carried over from elsewhere.

### Segmentation — `make segmentation`

25 scans along the aisle, ~28 500 points each, against per-point ground truth.

| Ground removal | |
|---|---|
| Recall | **1.0000** — no object point is ever discarded as floor |
| Precision | 0.9608 |
| False negatives | **0** |

| Clustering (per connected object) | |
|---|---|
| **People recovered** | **93.8%** (30 / 32) |
| Pallets recovered | 76.7% |
| Structure recovered | 65.9% |
| Spurious detections | **0** |

| Latency per scan | Median | p95 |
|---|---|---|
| Ground removal | 14.0 ms | 15.3 ms |
| Clustering | 13.9 ms | 17.2 ms |
| Occupancy grid | 6.3 ms | 7.4 ms |
| **Total** | **34.2 ms** | **38.7 ms** |

Budget is 100 ms at 10 Hz; 0 of 25 scans exceeded it. Details and the
optimisations that got there (ground removal 74 → 13.5 ms, clustering 121 →
8.9 ms) are in [docs/segmentation.md](docs/segmentation.md).

### LiDAR odometry against ground truth — `make scan-odometry`

A 24.2 m aisle traverse. Wheel odometry is simulated alongside with a 2% scale
error and two discrete wheel slips, because slip is exactly what the scan matcher
is in the stack to cover.

| | Wheel odometry | LiDAR odometry | |
|---|---|---|---|
| ATE RMSE | 0.574 m | **0.215 m** | −63% |
| ATE 95th percentile | 0.851 m | 0.385 m | |
| Yaw RMSE | 0.59° | **1.04°** | |
| RPE translation RMSE (1 s) | 0.0705 m | **0.0192 m** | −73% |

0.8% of matches rejected; 3.0 ms median per match.

**Read this table next to the simulated history, because they disagree.** In
the Gazebo Classic corridor the same code tracked nothing at all, so it ships
wired into the EKF and disabled; the diagnosis is in
[docs/localisation-accuracy.md](docs/localisation-accuracy.md). It has not been
re-measured on gz-sim.

Both tables reproduce **identically** on Windows (Python 3.9, numpy 1.26,
scipy 1.13), WSL Ubuntu 22.04 (Python 3.10, numpy 1.21, scipy 1.8) and a clean
`python:3.10-slim` container (numpy 2.2, scipy 1.15) — every stochastic step
takes a fixed seed, so the pipeline is deterministic end to end. That span
includes the numpy 1.x to 2.x boundary, which is why `requirements.txt` is
unpinned: pinning would hide it if that ever stopped being true.
Latency differs, as it should; the slower machine's figures are the ones quoted.

### End-to-end on gz-sim

Recorded on 2026-10-30 on gz-sim 8 (Harmonic) with ROS 2 Jazzy, against the
simulator's ground truth. The tables are rendered from
[`results/reference/gazebo_localisation.json`](results/reference/gazebo_localisation.json)
and CI fails if they stop matching it (`tests/test_results_reference.py`).

**Localisation with Nav2 out of the loop** (`make openloop`: a fixed velocity
profile, 12.0 m over 60 s, so the result measures the estimators and not the
controller):

<!-- gz-results:openloop -->
| Source | ATE RMSE | ATE max | Yaw RMSE | Path vs truth | Mean NEES |
|---|---|---|---|---|---|
| Wheel odometry | 0.001 m | 0.002 m | 0.38 deg | -0.1% | 0.00 |
| EKF fused | 0.003 m | 0.005 m | 0.08 deg | -0.1% | 0.00 |
| AMCL | 0.101 m | 0.156 m | 0.37 deg | +3.0% | 1.75 |
<!-- /gz-results:openloop -->

**The full mission** (`make eval`: six waypoints along the aisle and back, Nav2
in the loop, one goal at a time with a 2 s dwell at each). It **succeeded**:
6 of 6 waypoints in 139.8 s of simulated time, 49.3 m by ground truth, with no
recovery needed.

<!-- gz-results:mission -->
| Source | ATE RMSE | ATE max | Yaw RMSE | Path vs truth | Mean NEES |
|---|---|---|---|---|---|
| Wheel odometry | 0.031 m | 0.079 m | 0.13 deg | +0.0% | 0.97 |
| EKF fused | 0.011 m | 0.030 m | 0.09 deg | +0.0% | 0.00 |
| AMCL | 0.057 m | 0.121 m | 0.34 deg | -0.2% | 0.34 |
<!-- /gz-results:mission -->

**Read these with one caveat: on gz-sim the wheels do not slip.** DART's wheel
contact tracks ground truth to millimetres over the open-loop profile, where
Classic's ODE contact had drifted 0.48 m, so the EKF has almost nothing to
correct and AMCL is the largest error in the stack (about 0.1 m, trailing the
robot in the direction of travel). That is a property of the simulator, not a
result about the estimators: a deliberate encoder error model is the honest fix
and is listed in [STATUS.md](STATUS.md). Zero jumps over 0.5 m in either run;
in racking that repeats every 2.7 m that is the property that matters most.

The Gazebo Classic figures that tuned this stack (AMCL 0.077 m open loop,
0.119 m over a 56 m mission) are kept, labelled, in
[docs/localisation-accuracy.md](docs/localisation-accuracy.md).

### Recovery under induced failure, in CI

Every push runs `src/amr_bringup/test/test_navigation_recovery.launch.py`
headless on gz-sim: the robot localises against the committed map and drives
10.65 m down the aisle, and ten seconds in a pallet is set down on its goal for
45 s. The planner still finds a path (the LiDAR only sees the pallet's near
face), the controller cannot follow it, and the behaviour tree has to walk its
recovery ladder until the pallet is lifted. The test asserts that the mission
succeeds, that the fault fired and cleared, that a recovery from the ladder
ran, that the robot ends within 0.35 m of the goal by ground truth, and that
AMCL tracked ground truth (ATE RMSE under 0.30 m, no jump over 0.5 m).

The five-fault recovery matrix (`make faults`) was measured under Gazebo
Classic and has been ported but not rerun; it is in
[docs/recovery-matrix.md](docs/recovery-matrix.md), labelled as such.

### What moving to gz-sim found

| Finding | Effect |
|---|---|
| DART puts a cylinder's floor contacts at its rim edges | The 50 mm tyres' inner edges carried the load: effective track 0.45 m against 0.50 m, every in-place turn 11% too far (397 degrees for a commanded 360). Wheel collision is now a 4 mm disc |
| DWB's `RotateToGoal` reads `FollowPath.xy_goal_tolerance`, not `RotateToGoal.xy_goal_tolerance` | The tuned value was never read; the 0.25 m default against a 0.20 m goal checker stopped the robot 0.24 m short of every goal, with all 779 sampled forward trajectories rejected |
| tf2 0.36.22 deadlocks `waitForTransform` against `testTransformableRequests` | AMCL and both costmaps froze a minute or two into a run. Found with a `gdb` backtrace; fixed upstream in 0.36.23, which the image and CI now require |
| A U-turn handed to Nav2 as one path | Outbound and return legs overlap in DWB's window and the robot circled 2 m short of the turn. The mission runner now sends one goal at a time |
| `set_pose` does not move a static model | A teleported pallet stayed invisible to both LiDARs; faults now move a dynamic pallet |
| `gz.msgs.Odometry` has no covariance | The EKF would have trusted the wheels absolutely; `odom_covariance` restores the Classic values |

Detail in [docs/simulation.md](docs/simulation.md).

### The decoupling that made this measurable (Gazebo Classic)

A Nav2 mission cannot be used to tune localisation. A localisation error makes
the costmap wrong, Nav2 runs recovery spins, the robot clips something, the
wheels slip, and the slip corrupts the odometry localisation depends on.

Wheel odometry "over-reported by 276%", then 169%, then 56%, then 36% across
successive mission runs. In the open-loop harness, same code, no Nav2, it
over-reports by **0%**. None of those were sensor faults — the robot was pinned
against a pallet with its wheels turning.

### Five defects that only a system test could find (Gazebo Classic)

| Bug | Effect |
|---|---|
| `gazebo_ros_diff_drive` publishes an **unsigned** wheel twist (`sqrt(dx²+dy²)`) | Every reverse integrated as a forward. EKF ATE 6.72 m → 0.157 m when fixed |
| IMU `ax` fused with a 0.1 m/s² bias, no bias estimation | Constant acceleration integrated into position |
| Trimmed ICP discards the racking uprights as outliers | In a corridor of parallel walls those are the only points constraining motion; at `trim_ratio` 0.80 the matcher lost half the motion, at 0.60 all of it |
| `inflation_radius: 0.65`, sized for an **empty** aisle | A person standing in the middle of a 2.8 m aisle is inflated from one side and the racking from the other. The two met, **closing the aisle by 0.15 m**. No zero-cost route past them existed, and NavFn ended the run with `failed to create plan`. The robot stalled 8 cm from the person for 21 s, wheels turning, which fed AMCL 0.79 m of motion that never happened |
| AMCL `z_rand: 0.5` — half the measurement weight uniform | A uniform term awards the same likelihood to every hypothesis however badly it fits, so in a map repeating every 2.7 m the right bay could not be told from the wrong one. AMCL 1.272 m → **0.077 m** |

Full diagnosis, an experiment that made AMCL six times worse, and the
outstanding work are in
[docs/localisation-accuracy.md](docs/localisation-accuracy.md).

---

## Fleet interface: VDA 5050 over MQTT

A warehouse robot is dispatched by a fleet manager, not driven from RViz.
`amr_fleet` is the vehicle side of VDA 5050 2.0, the intralogistics standard for
that link: it subscribes to `uagv/v2/CGainsWorks/amr-001/order`, drives each
released node with Nav2, and publishes `state` every second and on every change
(`connection` carries ONLINE and the MQTT last will). The protocol logic has no
ROS or MQTT imports and is unit tested offline.

It implements the subset a single order needs and refuses the rest with a VDA
error rather than ignoring it: released and horizon nodes, traversal reporting,
validation, `cancelOrder` and every required state field. Node and edge actions,
order updates, the factsheet and visualisation are not implemented, and the
battery is a labelled constant because nothing simulates one. CI publishes a
three-node order over a real Mosquitto broker and waits for the state topic to
report it complete. Details and an example order are in
[docs/fleet-interface.md](docs/fleet-interface.md).

---

## What CI runs

| Job | What it does |
|---|---|
| Offline checks (no ROS) | flake8, 267 unit tests, generated files up to date, segmentation and scan-odometry benchmarks, README against the recorded simulation results |
| Jazzy + gz-sim Harmonic | builds the workspace in `ros:jazzy-ros-base-noble`, checks the URDF and launch files, then runs both integration tests headless (gz-sim with CPU rendering through Mesa llvmpipe): the recovery mission above and the VDA 5050 order. Summaries are uploaded as the `integration-results` artifact |

---

## What is in here

| | |
|---|---|
| **gz-sim warehouse** | 30 × 12 m, 10 racking bays, generated from one set of constants alongside the zone definitions and the reference map; gz-sim Harmonic on ROS 2 Jazzy through `ros_gz_bridge` |
| **Cartographer SLAM** | 2D mapping, tuned against bay aliasing |
| **AMCL** | localising against the saved map; random-particle injection disabled, on measured evidence |
| **EKF** | `robot_localization` fusing wheel odometry and IMU yaw rate. Scan odometry is wired in and **off by default** — it degrades the estimate in simulation |
| **Scan odometry** | keyframed trimmed ICP, written here rather than vendored |
| **Nav2** | NavFn + DWB, every costmap and DWB parameter justified |
| **Recovery** | a 5-step ladder, exercised by an induced failure in CI on every push (and by 5 induced failures under Gazebo Classic) |
| **Segmentation** | RANSAC ground removal with a seed band and normal constraint; range-adaptive Euclidean clustering |
| **Zone occupancy** | 7 work zones, point-count based, asymmetric hysteresis |
| **Evaluation** | ATE / RPE / NEES against simulator ground truth, a per-source path-length audit, and an open-loop harness that decouples localisation from navigation |
| **Fleet interface** | VDA 5050 2.0 over MQTT: orders in, state out, a Mosquitto broker in compose |

```
src/amr_common/        rigid-body maths, no ROS imports
src/amr_msgs/          DetectedObject, ZoneOccupancy
src/amr_description/   URDF, gz-sim systems and sensors
src/amr_localisation/  Cartographer, AMCL, EKF, ICP scan odometry
src/amr_navigation/    Nav2 params, behaviour trees
src/amr_perception/    ground removal, clustering, zones, occupancy
src/amr_bringup/       launch, gz bridge, mission runner, pose logger, fault injector,
                       the navigation integration test
src/amr_fleet/         VDA 5050 order and state logic, MQTT adapter, its integration test
simulation/            world generator, worlds, zones, reference map
analysis/              localisation error, segmentation metrics, recovery matrix,
                       the recorded simulation results reference
results/reference/     the one tracked result: the simulated runs the README quotes
tests/                 267 tests, a few seconds, no ROS required
```

Every algorithm lives in a `core/` subpackage that imports nothing from ROS. That
is what lets the whole stack be tested and profiled on a laptop, and it is why
`analysis/scan_odometry_benchmark.py` measures the *shipped* class rather than a
copy of it.

---

## Three things worth reading the code for

**The benchmark found a real bug.** The first working scan odometry reported
5.76 m ATE and 30° of yaw error while showing healthy fitness on almost every
frame. `converged` was computed and never checked, and `fitness` was defined in a
way that could not fall below `trim_ratio` no matter how wrong the match was. The
diagnosis, the three fixes and the regression tests are in
[docs/localisation-accuracy.md](docs/localisation-accuracy.md).

**A metric that measured the wrong thing.** Clustering initially scored 21%
instance recovery. The metric was wrong, not the code: it treated every rack
upright, beam and carton as a separate target, and a Euclidean clusterer cannot
by definition separate geometry that touches. Scored against connected
components — the question the algorithm actually answers — the same code reports
74%. See [docs/segmentation.md](docs/segmentation.md).

**A fixed clustering radius is a range limit in disguise.** At 2° between LiDAR
rings the vertical spacing is `0.035 × range`, so 0.42 m at 12 m — larger than
the 0.35 m tolerance. A person 11 m away produced 41 returns that fragmented into
per-ring slices and was detected as *nothing*. The range-adaptive merge took
people from 62.5% to 93.8%.

---

## Reproducing everything

| Command | Needs | Produces |
|---|---|---|
| `make test` | Python | 267 unit tests |
| `make segmentation` | Python | `results/segmentation.json` |
| `make scan-odometry` | Python | `results/scan_odometry.json` |
| `make check` | Python | all of the above |
| `make world` | Python | regenerates world, zones, reference map |
| `make demo` | ROS 2 Jazzy + gz-sim | the simulation, with the gz GUI and RViz |
| `make map` | ROS 2 Jazzy + gz-sim | a Cartographer map |
| `make eval` | ROS 2 Jazzy + gz-sim | the six-waypoint mission and its localisation accuracy |
| `make openloop` | ROS 2 Jazzy + gz-sim | localisation accuracy, Nav2 out of the loop |
| `make faults` | ROS 2 Jazzy + gz-sim | the recovery behaviour matrix |
| `make integration` | ROS 2 Jazzy + gz-sim | the two integration tests CI runs |

Docker equivalents: `make docker-check`, `make docker-demo`, or
`docker compose -f docker/docker-compose.yml run --rm eval` (and `integration`,
`fleet`). After `make openloop` and `make eval`,
`python3 analysis/results_reference.py build ...` rebuilds the reference file
and `render` prints the README tables.

**Requirements:** ROS 2 Jazzy with gz-sim Harmonic for the simulated targets,
the officially paired long-term combination (Jazzy to May 2029, Harmonic to
September 2028). Gazebo Classic 11, which this stack was built on, reached end
of life in January 2025, and Humble's partner Fortress in September 2026; the
reasoning and what changed are in [docs/simulation.md](docs/simulation.md).
`ros-jazzy-ros-gz` brings Harmonic with it. tf2 0.36.23 or later is required.
`./scripts/setup.sh` sets up a native workspace; Docker is the supported path.

---

## Documentation

| | |
|---|---|
| [STATUS.md](STATUS.md) | what is complete, in progress and not started, and the limitations |
| [architecture.md](docs/architecture.md) | frame ownership, data flow, why each algorithm was chosen |
| [simulation.md](docs/simulation.md) | why Jazzy with Harmonic, what was kept equivalent, what the migration found |
| [fleet-interface.md](docs/fleet-interface.md) | the VDA 5050 interface, its subset, and how to drive it |
| [parameter-rationale.md](docs/parameter-rationale.md) | every costmap, DWB, AMCL, EKF and Cartographer value, with derivations |
| [localisation-accuracy.md](docs/localisation-accuracy.md) | metrics, measured results, and the bug the benchmark found |
| [segmentation.md](docs/segmentation.md) | dataset choice, accuracy, latency, zone design |
| [recovery-matrix.md](docs/recovery-matrix.md) | the recovery ladder and the induced failures |
| [limitations.md](docs/limitations.md) | what this does not do, and what has not been run |

---

## Licence

MIT — see [LICENSE](LICENSE).

No third-party datasets are used. The segmentation benchmark scene is generated
by code in this repository, so there is no dataset licence to record.
