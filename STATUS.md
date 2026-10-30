# Status

Last updated: 2026-10-30. Everything simulated here is simulated: there is no
robot hardware behind this repository, and nothing in it claims otherwise.

## Complete

* **Offline pipeline, no ROS.** RANSAC ground removal, range-adaptive Euclidean
  clustering, voxel occupancy, debounced zone occupancy, keyframed ICP scan
  odometry and the error metrics, against a ray-traced warehouse scene. 267 unit
  tests; CI runs them and both benchmarks on every push.
* **Migration to gz-sim Harmonic on ROS 2 Jazzy**, the officially paired
  long-term combination, from Gazebo Classic 11 (end of life January 2025).
  World, robot and sensors kept equivalent; `ros_gz_bridge` carries every topic.
  Reasoning and findings in [docs/simulation.md](docs/simulation.md).
* **Localisation and navigation on gz-sim.** Cartographer (mapping), AMCL
  against the committed map, the EKF and Nav2 (NavFn, DWB, recovery ladder).
  The open-loop harness and the full six-waypoint mission were recorded on
  2026-10-30; the mission succeeds (6 of 6 waypoints, 139.8 s simulated, no
  recovery needed). Numbers in the README, from
  `results/reference/gazebo_localisation.json`, which CI checks the README
  against.
* **Headless integration tests in CI** (launch_testing, gz-sim with CPU
  rendering): a mission with an induced failure (a pallet set down on the
  goal) that must exercise a recovery and still succeed, with ground-truth and
  AMCL bounds asserted.
* **VDA 5050 fleet interface** over MQTT, with Mosquitto in compose and an
  integration test in CI that sends an order and observes completion in
  simulation. The implemented subset is stated in
  [docs/fleet-interface.md](docs/fleet-interface.md).

## In progress

* Nothing is mid-change in this repository.

## Not started

* **Rerunning `make map` and `make faults` on gz-sim.** Both are ported, neither
  has been rerun since the migration. The recovery matrix in
  [docs/recovery-matrix.md](docs/recovery-matrix.md) is a Gazebo Classic
  result.
* **A wheel-encoder error model.** On gz-sim the wheels do not slip, so wheel
  odometry is near-ideal and the EKF has little to correct. Scale error and
  slip events, as the offline scan-odometry benchmark already simulates, would
  make the comparison meaningful again.
* **Finding why AMCL trails the robot by about 0.1 m on gz-sim.** It is not
  LiDAR timestamp latency (checked), and it is within the bounds CI asserts.
* **Re-measuring scan odometry on gz-sim.** It ships disabled on the strength
  of the Classic measurements.
* **The rest of VDA 5050:** node and edge actions, order updates and stitching,
  `factsheet`, `visualization`.
* **A demo GIF or video**, and a tracker on clusters so detected people have
  velocities.

## Limitations

* Simulation only. Real-robot bring-up (drivers, calibration, a real LiDAR's
  reflectivity and multipath) has not been done and would be needed before any
  number here describes a vehicle.
* Single robot. The VDA 5050 interface lets a master control dispatch it, but
  there is no fleet manager, traffic management or multi-robot deadlock handling
  here.
* Both LiDARs are rendered on the CPU without a GPU, so the simulation runs
  below real time; all results are in simulated time.
* Simulated runs are not bit reproducible across machines. CI asserts bounds on
  its own runs and checks the README against the recorded file, rather than
  diffing numbers.
* The battery is not simulated; the VDA 5050 state reports a labelled constant.
* tf2 0.36.23 or later is required (0.36.22 deadlocks AMCL and the costmaps).

The full list, with the reasoning behind each, is in
[docs/limitations.md](docs/limitations.md).
