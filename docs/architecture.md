# Architecture

## Frame ownership

Everything else follows from this, so it comes first.

```
map ──────────── AMCL ────────────► odom ──── robot_localization EKF ───► base_link ──► base_footprint
 │                                    │                                       │
 │ static map                         │ wheel odom + IMU + scan odom          │ fixed joints
 │ (from Cartographer)                │ 30 Hz, continuous, no jumps           │
```

`map → odom` is AMCL's. `odom → base_footprint` is the EKF's. Nothing else
publishes on either edge, and `tests/test_config_consistency.py` asserts it —
including that the Gazebo differential-drive plugin has `publish_odom_tf` false,
which is the easiest way to end up with two publishers fighting over one
transform.

### The EKF must publish to the URDF *root*

`base_footprint`, not `base_link`. This is one word in `ekf.yaml` and it cost a
whole simulation run to find, so it is worth spelling out.

The URDF roots at `base_footprint` and `robot_state_publisher` owns the fixed
edge `base_footprint → base_link`. Point the EKF at `base_link` and it publishes
`odom → base_link` as well, so `base_link` has two parents. That is not a tree:
tf2 silently splits into two disconnected components and every lookup spanning
them fails.

What makes it expensive is that the symptom points nowhere near the cause. What
you actually see is:

* AMCL: `Could not find a connection between 'base_footprint' and
  'lidar_2d_link' because they are not part of the same tree.`
* `map` frame never appears at all, so anything asking for `map → base_footprint`
  waits forever.
* `/amcl_pose` never publishes.
* The segmentation node goes silent — it drops every scan on a failed transform
  lookup, exactly as designed, but the log line looks like a perception problem.

Four subsystems appear broken; one word is wrong. The invariant is now asserted
two ways: `test_ekf_base_frame_is_the_urdf_root` derives the root from the URDF
and compares it, and `test_base_frame_agrees_across_ekf_amcl_and_gazebo` checks
the EKF, AMCL and the Gazebo plugin all name the same frame.

Worth noting what behaved *correctly* through all of this: the zone publisher
kept publishing with `transform_valid: false` rather than reporting its zones
clear. That is the designed failure mode working — it had lost the ability to
place its zones and said so, instead of asserting the aisle was empty.

The split matters because the two estimates have different jobs. The EKF gives a
locally smooth, continuously differentiable pose that DWB can differentiate
without seeing steps. AMCL corrects the accumulated drift in discrete jumps. Let
AMCL publish `odom → base_link` directly and the controller sees a velocity
discontinuity every time the particle filter resamples.

## Data flow

```
                 ┌──────────────────────────── Gazebo ───────────────────────────┐
                 │  2D LiDAR    3D LiDAR    IMU    wheel encoders    p3d truth   │
                 └──────┬──────────┬─────────┬───────────┬────────────────┬──────┘
                        │          │         │           │                │
              /scan ────┤          │  /imu/data     /odom │                │ /ground_truth/pose
                        │          │         │           │                │
        ┌───────────────┼──────────┼─────────┼───────────┤                │
        │               │          │         │           │                │
        ▼               ▼          │         │           │                ▼
   scan_odometry    segmentation   │         │           │           pose_logger
    (ICP, 10 Hz)     (10 Hz)       │         │           │          (evaluation only)
        │               │          │         │           │
        │ /scan_odom    │ /obstacles          │           │
        │               │          │         │           │
        ▼               │          ▼         ▼           │
   ┌─────────────────────────────────────────────┐       │
   │   robot_localization EKF   (odom1, imu0, odom0)     │
   └──────────────────────┬──────────────────────┘       │
                          │ /odometry/filtered           │
                          ▼                              │
                        AMCL ◄───────────────────── /scan
                          │ /amcl_pose, map→odom
                          ▼
   ┌──────────────────────────────────────────────────────────┐
   │  Nav2: global costmap → NavFn → DWB → velocity smoother   │
   └──────────────────────────────────────────────────────────┘
                          │ /cmd_vel
                          ▼
                       Gazebo

   segmentation ──/obstacles──► zone_occupancy ──► /zones (per-zone booleans)
                └──/obstacles──► local costmap obstacle layer
```

`pose_logger` is the only subscriber to `/ground_truth/pose`, and it checks that
at runtime. See [limitations.md](limitations.md).

## Packages

| Package | Type | Contains |
|---|---|---|
| `amr_common` | ament_python | Quaternion and SE(2) helpers. No ROS imports. |
| `amr_msgs` | ament_cmake | `DetectedObject[Array]`, `ZoneOccupancy[Array]`. |
| `amr_description` | ament_cmake | URDF/xacro, Gazebo plugins, RViz config. |
| `amr_localisation` | ament_python | Cartographer, AMCL, EKF config; ICP scan odometry. |
| `amr_navigation` | ament_cmake | Nav2 params, behaviour trees. |
| `amr_perception` | ament_python | Ground removal, clustering, zones, occupancy. |
| `amr_bringup` | ament_python | Launch, mission runner, pose logger, fault injector. |

### Two deviations from the layout in the brief

**`amr_common` was added.** Three consumers need identical rigid-body maths:
`amr_perception` (moving clouds between frames), `amr_localisation` (composing
SE(2) poses) and the offline scripts in `analysis/` (recovering yaw from logged
quaternions). The alternative was three copies of one quaternion convention,
which is how one of them acquires a sign error that nobody notices until a cloud
appears mirrored in RViz. It has no ROS dependency, so `analysis/` can import it
without a sourced workspace.

**`amr_msgs` was added.** The zone publisher needs to emit a consistent snapshot
across all zones on one stamp. A topic per zone gives consumers a set of
independently-timed booleans that can disagree with each other, which is the
wrong interface for something used to decide whether it is safe to enter an
aisle.

## The core/node split

Every algorithm lives in a `core/` subpackage that imports nothing from ROS.
The ROS nodes are thin: convert messages, look up a transform, call into the
core, publish.

This is a hard rule, not a stylistic preference, and it is what makes the rest of
the repository possible:

* The whole segmentation and scan-matching stack is unit tested on a laptop with
  numpy and scipy and nothing else — 171 tests in under four seconds.
* CI runs the real algorithms rather than a mock of them.
* `analysis/scan_odometry_benchmark.py` measures the *shipped* `ScanOdometry`
  class, not a reimplementation of it. A benchmark that reimplements what it
  measures eventually disagrees with it, and the disagreement is discovered by
  the robot.

## Why these algorithm choices

**NavFn over Smac.** The warehouse is topologically a corridor: there is
essentially one route between any two points. The sophistication Smac buys —
cost-aware smoothing across open floor, kinematically feasible curves — has
nothing to work with here, and NavFn plans the full 30 m in single-digit
milliseconds. Smac 2D would be the right choice if there were open cross-dock
areas where several routes compete.

**Likelihood field over beam model in AMCL.** The beam model ray-traces every
beam against the map for every particle, which at 2000 particles does not fit in
a 10 Hz budget.

**A hand-written ICP rather than a scan-matching package.** The two obvious
candidates, `rf2o_laser_odometry` and `scan_tools/laser_scan_matcher`, are both
source-only on Humble. Vendoring a build of either to obtain roughly 150 lines of
arithmetic is a poor trade, and a self-contained implementation can be measured
for accuracy directly — which is what found a real bug (see
[localisation-accuracy.md](localisation-accuracy.md)).

**A dense boolean voxel grid rather than OctoMap.** Log-odds is the right
structure for accumulating evidence over a long traverse while distinguishing
"known free" from "unknown". The zone publisher asks a much narrower question —
which voxels have current returns — at 10 Hz, and a dense grid over a bounded
region answers it in 6 ms with none of the tuning.

**A generated world.** `simulation/generate_world.py` emits the Gazebo world,
the zone definitions and the reference map from one set of constants. The same
constants drive `amr_perception.core.synthetic`, the ray-traced scene the
offline metrics are measured on. A hand-maintained SDF gets edited, the synthetic
scene does not, and then the offline numbers quietly stop describing the
simulated warehouse. CI asserts the generated files are current.
