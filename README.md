# ros2-warehouse-amr-sim

An autonomous mobile robot navigation stack for a simulated warehouse, plus a 3D
LiDAR segmentation pipeline that produces occupancy state for defined work zones.
Built entirely in Gazebo, demonstrating the same architecture as an industrial
AMR deployment.

> ### Clean-room reimplementation
> This repository is an **independent reimplementation of a problem class**, built from
> scratch on personal time using public or synthetic data and open-source simulation.
> It is **not** derived from, and contains no material belonging to, any employer or
> client: no employer source code, schematics, calibration data, client data or internal
> documentation. It exists to demonstrate the architecture and engineering approach only.

---

```
map ──────── AMCL ────────► odom ──── EKF (wheel + IMU + LiDAR) ───► base_link
                              ▲
   ┌──────────────────────────┴───────────────────────────┐
   │                                                       │
2D LiDAR ──► scan matcher (ICP, keyframed) ────────────────┤
   │                                                       │
   └──► AMCL ──► Nav2: NavFn ► DWB ► velocity smoother ► /cmd_vel
                                                           │
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
pip install numpy scipy pyyaml pytest
make check
```

```bash
# The full simulation. Needs Docker.
make docker-demo
```

`make check` runs the real algorithms — RANSAC ground removal, Euclidean
clustering, ICP scan matching, the error metrics — against a ray-traced warehouse
scene, with no ROS and no simulator. It is what CI runs on every push.

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

**Read this table next to the simulated one below, because they disagree.** The
same code that scores a 63% improvement here tracks nothing at all in the
simulated corridor, and the simulated result is the one that counts.

Both tables reproduce **identically** on Windows (Python 3.9, numpy 1.26,
scipy 1.13) and WSL Ubuntu 22.04 (Python 3.10, numpy 1.21, scipy 1.8) — every
stochastic step takes a fixed seed, so the pipeline is deterministic end to end.
Latency differs, as it should; the slower machine's figures are the ones quoted.

### End-to-end in Gazebo

**Localisation, measured with the navigation stack out of the loop** — a fixed
velocity profile, so the result measures the estimator and not the controller:

| Source | ATE RMSE | Yaw RMSE | Covariance |
|---|---|---|---|
| Wheel odometry | 0.482 m | 24.4° | — |
| EKF fused | 0.183 m | 4.63° | consistent |
| **AMCL** | **0.077 m** | **1.18°** | **consistent** |

Both filters started above 6.6 m. Both are now statistically consistent, and
AMCL is the best estimator in the stack — which is what it is for, since
correcting the EKF's drift is its whole job.

**The full mission, end to end** — six waypoints, 56 m, Nav2 in the loop:

| Source | ATE RMSE | max | Yaw RMSE | Path vs truth | Covariance |
|---|---|---|---|---|---|
| Wheel odometry | 2.788 m | 7.243 m | 2.01° | +0% | — |
| EKF fused | 0.320 m | 0.800 m | 0.33° | +0% | consistent |
| **AMCL** | **0.119 m** | **0.226 m** | **0.79°** | +1% | consistent (NEES 2.05) |

**Zero discontinuities over 0.5 m.** In a map whose racking repeats every 2.7 m,
a localiser that never once jumps a bay across 56 m is the result that matters
more than the RMSE.

Wheel odometry drifts 2.8 m over 56 m and that is *correct behaviour* — dead
reckoning with a 5% scale error is what it is for. What matters is that its path
length is within 0% of ground truth: **no slip anywhere in the run**. Earlier
missions over-reported by 276%, 169%, 56% and 36%, every one of them a robot
pinned against something with its wheels turning.

### The decoupling that made this measurable

A Nav2 mission cannot be used to tune localisation. A localisation error makes
the costmap wrong, Nav2 runs recovery spins, the robot clips something, the
wheels slip, and the slip corrupts the odometry localisation depends on.

Wheel odometry "over-reported by 276%", then 169%, then 56%, then 36% across
successive mission runs. In the open-loop harness, same code, no Nav2, it
over-reports by **0%**. None of those were sensor faults — the robot was pinned
against a pallet with its wheels turning.

### Five defects that only a system test could find

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

## What is in here

| | |
|---|---|
| **Gazebo warehouse** | 30 × 12 m, 10 racking bays, generated from one set of constants alongside the zone definitions and the reference map |
| **Cartographer SLAM** | 2D mapping, tuned against bay aliasing |
| **AMCL** | localising against the saved map; random-particle injection disabled, on measured evidence |
| **EKF** | `robot_localization` fusing wheel odometry and IMU yaw rate. Scan odometry is wired in and **off by default** — it degrades the estimate in simulation |
| **Scan odometry** | keyframed trimmed ICP, written here rather than vendored |
| **Nav2** | NavFn + DWB, every costmap and DWB parameter justified |
| **Recovery** | a 5-step ladder, exercised by 5 induced failures |
| **Segmentation** | RANSAC ground removal with a seed band and normal constraint; range-adaptive Euclidean clustering |
| **Zone occupancy** | 7 work zones, point-count based, asymmetric hysteresis |
| **Evaluation** | ATE / RPE / NEES against Gazebo ground truth, a per-source path-length audit, and an open-loop harness that decouples localisation from navigation |

```
src/amr_common/        rigid-body maths, no ROS imports
src/amr_msgs/          DetectedObject, ZoneOccupancy
src/amr_description/   URDF, Gazebo plugins
src/amr_localisation/  Cartographer, AMCL, EKF, ICP scan odometry
src/amr_navigation/    Nav2 params, behaviour trees
src/amr_perception/    ground removal, clustering, zones, occupancy
src/amr_bringup/       launch, mission runner, pose logger, fault injector
simulation/            world generator, worlds, zones, reference map
analysis/              localisation error, segmentation metrics, recovery matrix
tests/                 197 tests, ~4 s, no ROS required
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
| `make test` | Python | 197 unit tests |
| `make segmentation` | Python | `results/segmentation.json` |
| `make scan-odometry` | Python | `results/scan_odometry.json` |
| `make check` | Python | all of the above |
| `make world` | Python | regenerates world, zones, reference map |
| `make demo` | ROS 2 + Gazebo | the simulation, with RViz |
| `make map` | ROS 2 + Gazebo | a Cartographer map |
| `make eval` | ROS 2 + Gazebo | localisation accuracy vs ground truth |
| `make openloop` | ROS 2 + Gazebo | localisation accuracy, Nav2 out of the loop |
| `make faults` | ROS 2 + Gazebo | the recovery behaviour matrix |

Docker equivalents: `make docker-check`, `make docker-demo`, or
`docker compose -f docker/docker-compose.yml run --rm eval`.

**Requirements:** ROS 2 Humble and Gazebo Classic 11 for the simulated targets.
That pairing is deliberate and pinned — the plugins used are Classic ones whose
Ignition equivalents have different names and parameters. `./scripts/setup.sh`
sets up a native workspace; Docker is the supported path.

---

## Documentation

| | |
|---|---|
| [architecture.md](docs/architecture.md) | frame ownership, data flow, why each algorithm was chosen |
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
