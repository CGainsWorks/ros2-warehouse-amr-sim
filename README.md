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
| ATE RMSE | 0.574 m | **0.105 m** | −82% |
| ATE 95th percentile | 0.851 m | 0.136 m | |
| Yaw RMSE | 0.59° | **0.12°** | −80% |
| RPE translation RMSE (1 s) | 0.0705 m | **0.0174 m** | −75% |

0.5% of matches rejected; 3.8 ms median per match.

### Not yet measured

**AMCL and EKF accuracy against Gazebo ground truth has not been run.** The
pipeline that produces it is implemented and tested, but no Gazebo run has been
performed, so no numbers are quoted. Generate them with `make eval`. Likewise the
recovery matrix — the faults and the ladder are implemented, the run is not done.
See [docs/limitations.md](docs/limitations.md).

---

## What is in here

| | |
|---|---|
| **Gazebo warehouse** | 30 × 12 m, 10 racking bays, generated from one set of constants alongside the zone definitions and the reference map |
| **Cartographer SLAM** | 2D mapping, tuned against bay aliasing |
| **AMCL** | localising against the saved map, with recovery injection enabled |
| **EKF** | `robot_localization` fusing wheel odometry, IMU and LiDAR scan odometry |
| **Scan odometry** | keyframed trimmed ICP, written here rather than vendored |
| **Nav2** | NavFn + DWB, every costmap and DWB parameter justified |
| **Recovery** | a 5-step ladder, exercised by 5 induced failures |
| **Segmentation** | RANSAC ground removal with a seed band and normal constraint; range-adaptive Euclidean clustering |
| **Zone occupancy** | 7 work zones, point-count based, asymmetric hysteresis |
| **Evaluation** | ATE / RPE / NEES against Gazebo ground truth, with a leak check |

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
tests/                 171 tests, ~4 s, no ROS required
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
| `make test` | Python | 171 unit tests |
| `make segmentation` | Python | `results/segmentation.json` |
| `make scan-odometry` | Python | `results/scan_odometry.json` |
| `make check` | Python | all of the above |
| `make world` | Python | regenerates world, zones, reference map |
| `make demo` | ROS 2 + Gazebo | the simulation, with RViz |
| `make map` | ROS 2 + Gazebo | a Cartographer map |
| `make eval` | ROS 2 + Gazebo | localisation accuracy vs ground truth |
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
