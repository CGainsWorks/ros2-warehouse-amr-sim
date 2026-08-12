# Limitations

Stated plainly. Several of these are consequences of deliberate design choices
and are recorded here rather than hidden.

## What has and has not been run

Everything in this repository has now been executed. The figures in the docs
come from runs, not from review.

* **Offline, no ROS** — 197 unit tests, the segmentation benchmark and the
  scan-odometry benchmark. Reproduce identically on Windows and WSL.
* **Gazebo** — mapping, the full six-waypoint mission, the open-loop
  localisation harness, and five fault-injection runs.

What is still missing:

**No demo GIF or video.** Every claim is a number in a JSON file rather than
something you can watch.

**One machine, one simulator build.** ROS 2 Humble and Gazebo Classic 11 in the
supplied Docker image, on one laptop. Gazebo's ODE solver is not deterministic
across machines, and the contact-heavy moments — the pallet nudge, the recovery
reversals — are exactly where that shows. Expect the mission to complete and the
localisation figures to land in the same range; do not expect them to match to
three decimals.

**Scan odometry ships disabled.** It is wired into the EKF and off by default,
because it degrades the estimate in simulation while improving it by 63% on the
offline benchmark. Both results are real and the discrepancy is diagnosed in
[localisation-accuracy.md](localisation-accuracy.md), but it is not fixed.

**The kidnap fault is not survivable**, by an explicit and measured trade: random
particle injection is disabled because it made localisation six times worse in a
map that repeats every 2.7 m. See [recovery-matrix.md](recovery-matrix.md).

## Perception

**No tracking.** `DetectedObject.id` is reassigned every scan by descending point
count. It is stable within a message and meaningless across messages. There is no
data association, so no object velocities and no prediction — a person walking
towards the robot is a fresh detection every 100 ms. Nav2's costmap handles that
adequately for avoidance, but anything wanting intent estimation needs a tracker
this does not have.

**No classification.** Clusters are geometric. Nothing distinguishes a person
from a pallet; downstream consumers that care use dimensions and zone membership.
A `class` field that is always zero would be worse than no field.

**Connected objects cannot be separated.** A person standing within 0.35 m of a
rack face merges into the racking, and no connectivity-based clustering can do
otherwise. In the benchmark scene the person at (−3.0, 1.0) sits 0.10 m from an
upright and does merge. The zone publisher is designed around this: it counts raw
points inside the zone prism rather than trusting cluster instances, so a merged
person still triggers the zone.

**Distant objects fragment.** The range-adaptive radius pushes the limit from
about 10 m to about 17.6 m, where the 0.80 m cap takes effect. Beyond that,
objects fall apart into per-ring slices again. The cap is not removable: a larger
radius exceeds the 1.1 m clearance between the aisle centreline and a rack face
and starts merging people into racking, which is the failure the adaptation
exists to prevent.

**Ground removal keeps the bottom 6 cm of everything.** Points on the lowest
6 cm of a wall, upright or pallet are geometrically indistinguishable from floor
at the chosen threshold. Measured precision is 0.9608 for that reason. Harmless
here, because those objects extend well above the threshold and are captured
anyway.

**One ground plane per scan.** A ramp, a dock leveller or a significant floor
joint would be fitted as a single plane and the deviation classified as
obstacle. Fine for a flat warehouse floor; wrong for a site with grade changes.

## Localisation

**A scan matched onto the wrong racking bay can be geometrically perfect.** In a
long, uniform aisle a bay-offset match converges cleanly, reports high fitness
and shows a low residual, because one bay genuinely does look exactly like the
next. No local quality metric can separate those cases — the information is not
present in the two scans. Only AMCL's global map and Cartographer's pose graph
can resolve it.

In *this* warehouse the aisle is only about ten bays long, and the end walls,
roof columns and aisle pallets break the periodicity enough that a bay-offset
match scores 0.58 against the 0.60 fitness gate and fails to converge, so it is
caught. `test_a_bay_offset_seed_lands_a_bay_out_but_is_caught` pins that down.
**That is a property of this floorplan, not a general guarantee.** A longer or
more uniform aisle would defeat it.

**Rejected scan intervals are dropped, not estimated.** When a match fails the
gates the pose is held and nothing is published, leaving the interval to wheel
odometry and the IMU. That is deliberate — inventing the motion would launder
wheel odometry through a topic claiming to be LiDAR-derived — but it does mean
the scan-odometry pose lags after a rejection. Measured reject rate on the
benchmark traverse is 0.5%.

**The scan matcher is 2D.** It assumes the sensor stays in a plane. Valid for a
flat floor; a robot pitching over a threshold violates it.

**Cartographer's `min_score` is raised to 0.68 from 0.55.** This trades loop
closures for safety against bay aliasing. The cost is more residual drift in the
final map. The alternative — a map folded a whole bay short — is not
recoverable.

## Navigation

**Recovery attribution is by time window, not causation.** See
`docs/recovery-matrix.md`. Nothing in Nav2 tags a recovery with its cause.

**NavFn ignores the footprint.** Path safety comes entirely from the inflation
layer. Adequate in a corridor; a planner that reasons about the footprint would
be needed for tight non-axis-aligned manoeuvring.

**The inflation cost step at 0.65 m is real.** Cost drops from 76 to 0 at the
inflation boundary. Accepted deliberately; the reasoning is in
`docs/parameter-rationale.md`.

**No dynamic obstacle prediction.** DWB plans against the costmap as it is now.
A person walking across the aisle is avoided reactively, not anticipated.

**Single robot.** No fleet coordination, no traffic management, no deadlock
resolution for two AMRs meeting in a 2.8 m aisle. The charger-bay zone exists
partly to make the shape of that problem visible, but nothing acts on it.

## Simulation

**The shipped map is generated from geometry, not from SLAM.**
`simulation/maps/warehouse.pgm` is rasterised from the world's own constants so
that a fresh clone can run the demo without first sitting through a mapping run,
and so that a navigation problem can be isolated from a mapping one. It is
labelled as such in its own YAML header. `make map` produces the real
Cartographer map, which will differ by Cartographer's error — comparing them is
the point.

**People are static cylinders teleported on a schedule.** A `<plugin>`-driven
walker would move unrepeatably and make the recovery matrix impossible to
reproduce. The cost is that no genuinely continuous dynamic motion is simulated.

**Gazebo Classic 11, not Ignition.** The plugins used — `gazebo_ros_diff_drive`,
`gazebo_ros_ray_sensor`, `gazebo_ros_p3d` — are Classic. The Ignition equivalents
have different names and parameters. This pins the stack to ROS 2 Humble.

A ROS 2 Humble install is **not** sufficient on its own. A stock
`ros-humble-desktop` does not pull in Gazebo, and the following are separate
installs that this stack requires: `gazebo`, `ros-humble-gazebo-ros-pkgs`,
`ros-humble-robot-localization`, `ros-humble-cartographer-ros` and
`ros-humble-xacro`. `scripts/setup.sh` installs all of them; the Docker image
is the supported path precisely because that list is easy to get wrong.

**The casters are held 3 mm off the floor.** A physical AMR rests on wheels and
casters together. Here the drive wheels carry the load and the casters only stop
it pitching, because with both bottoming at exactly z = 0 the contact solver left
the drive wheels with no normal force at all — the wheels turned at exactly the
commanded rate and the robot did not move. This is a simulation artefact being
worked around, not a design choice, and the consequence is that the modelled
weight distribution is more wheel-biased than a real robot's would be. Traction
in simulation is therefore slightly optimistic.

**Simulated sensor noise is representative, not characterised.** 1 cm LiDAR
range noise and the IMU noise densities are typical of the hardware class this
targets; they are not measured from a specific unit. Numbers from this repository
describe this simulation, and transfer to hardware only as far as that model
holds.

## Evaluation

**Offline metrics use a synthetic scene, not recorded data.** Deliberate — see
`docs/segmentation.md` — but it means the segmentation numbers describe an exact
ray-traced world with no reflectivity effects, no multi-path, no dust, and no
mixed pixels at object edges. Real LiDAR has all four, and all four make
segmentation harder.

**The NEES verdict does not distinguish over-conservative from well-calibrated.**
`analysis/localisation_error.py` reports "consistent" for anything that is not
overconfident, so a filter whose covariance is far too *large* passes. The EKF on
the full mission reads a mean NEES of 0.00 against an expected 2.0 and is
reported as consistent. It is not lying about being wrong, but it is claiming far
more uncertainty than it has, and a downstream consumer that gated on the
covariance would be needlessly cautious. Splitting the verdict into
`overconfident` / `consistent` / `conservative` is a small change and has not
been made.

**NEES uses a diagonal covariance approximation.** The pose logger records only
the diagonal, so the cross-covariance is unavailable. This overstates NEES when x
and y errors are correlated, which in a corridor they are. The consistency check
is therefore conservative — it will call a filter overconfident slightly more
readily than the truth warrants.

**Latency figures are single-machine.** Measured on one core of a laptop CPU with
no other load. A real robot's compute has a costmap, a planner and a controller
competing for the same cores.
