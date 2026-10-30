#!/usr/bin/env bash
# Run the demo mission with pose logging, then report localisation accuracy.
#
# Sequencing note: the simulation is started in the background and the script
# waits for the map -> odom transform to exist before launching the mission,
# rather than sleeping for a fixed period. Gazebo's startup time varies by an
# order of magnitude between a warm and a cold machine, and a fixed sleep is
# either wasteful or flaky depending on which one you tuned it on.

set -euo pipefail

RESULTS="${1:-results}"
TIMEOUT_STARTUP="${TIMEOUT_STARTUP:-180}"
# 900 s. gz-sim runs at roughly 0.3-0.6x real time here, so a six-waypoint
# mission over ~50 m does not fit in 400 s even when navigation behaves.
TIMEOUT_MISSION="${TIMEOUT_MISSION:-900}"

mkdir -p "${RESULTS}"

cleanup() {
  local status=$?
  echo "--- shutting down"
  # Kill the whole process group: ros2 launch spawns children that do not
  # always die with their parent, and a leftover gz sim server keeps
  # publishing the old world on the same gz partition, which
  # makes the next run fail with a misleading error.
  [[ -n "${SIM_PID:-}" ]] && kill -- -"${SIM_PID}" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

# The ablation: USE_SCAN_ODOMETRY=false leaves the EKF on wheel odometry and the
# IMU alone. Running the evaluation both ways is what isolates the LiDAR
# odometry's end-to-end contribution, as opposed to the component benchmark in
# analysis/scan_odometry_benchmark.py.
USE_SCAN_ODOMETRY="${USE_SCAN_ODOMETRY:-false}"

echo "--- starting simulation (headless, scan odometry: ${USE_SCAN_ODOMETRY})"
setsid ros2 launch amr_bringup simulation.launch.py headless:=true \
  use_scan_odometry:="${USE_SCAN_ODOMETRY}" \
  > "${RESULTS}/simulation.log" 2>&1 &
SIM_PID=$!

echo "--- waiting for localisation to come up (up to ${TIMEOUT_STARTUP}s)"
deadline=$((SECONDS + TIMEOUT_STARTUP))
# timeout, so the deadline below is reachable: `ros2 topic echo --once` blocks
# forever on a topic that never appears.
until timeout 5 ros2 topic echo /amcl_pose --once > /dev/null 2>&1; do
  if (( SECONDS > deadline )); then
    echo "ERROR: /amcl_pose never appeared. Last 40 lines of the simulation log:" >&2
    tail -n 40 "${RESULTS}/simulation.log" >&2
    exit 1
  fi
  sleep 2
done
echo "--- localisation is up"

# Nav2 must actually be ACTIVE before a mission is worth sending. If
# bt_navigator failed to configure — a bad behaviour-tree path does exactly
# this — the lifecycle manager aborts the bringup, and the only visible symptom
# is a goal rejected in 0.4 s with nothing in the mission log to explain it.
# Checking here turns that into a named failure before any time is spent.
# The lifecycle manager activates seven servers in sequence, each of which
# builds its costmaps and waits on TF. Under Gazebo running below real time that
# takes well over a minute, so this deadline matches the localisation one rather
# than being a tight "should be quick" guess — a gate that fires on a healthy
# but slow startup is worse than no gate at all.
TIMEOUT_NAV2="${TIMEOUT_NAV2:-180}"
echo "--- waiting for Nav2 to activate (up to ${TIMEOUT_NAV2}s)"
deadline=$((SECONDS + TIMEOUT_NAV2))
until [[ "$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)" == active* ]]; do
  if (( SECONDS > deadline )); then
    echo "ERROR: bt_navigator never reached the active state." >&2
    echo "Lifecycle states:" >&2
    for n in map_server amcl controller_server smoother_server planner_server \
             behavior_server bt_navigator waypoint_follower velocity_smoother; do
      printf '  %-20s %s\n' "$n" "$(timeout 5 ros2 lifecycle get "/$n" 2>/dev/null || echo unavailable)" >&2
    done
    echo "Relevant errors:" >&2
    grep -iE '\[ERROR\]' "${RESULTS}/simulation.log" 2>/dev/null | sed 's/\[[0-9.]*\]//' \
      | sort -u | head -10 >&2
    exit 1
  fi
  sleep 3
done
echo "--- Nav2 active after ${SECONDS}s"

echo "--- running mission"
set +e
timeout "${TIMEOUT_MISSION}" ros2 launch amr_bringup evaluation.launch.py \
  results_dir:="${RESULTS}" > "${RESULTS}/mission.log" 2>&1
MISSION_STATUS=$?
set -e

if [[ ${MISSION_STATUS} -ne 0 ]]; then
  echo "WARNING: the mission exited with status ${MISSION_STATUS}." >&2
  echo "Pose logs are still written, so the accuracy report below is valid for" >&2
  echo "however far the robot got. See ${RESULTS}/mission.log." >&2
fi

# Path length per source first. It localises a fault to one link in the chain
# in a way ATE cannot: an EKF shorter than every input it fuses is a fusion
# fault, an input that is itself short is a sensor-pipeline fault, and the two
# need different fixes.
echo "--- odometry audit"
python3 analysis/odometry_audit.py --results "${RESULTS}"   --output "${RESULTS}/odometry_audit.json"

echo
echo "--- localisation accuracy"
python3 analysis/localisation_error.py \
  --estimate "${RESULTS}/amcl_pose.csv" \
  --reference "${RESULTS}/ground_truth.csv" \
  --output "${RESULTS}/localisation_error.json" \
  --title "AMCL against Gazebo ground truth"

echo
# --align, because wheel odometry is in the odom frame whose origin is the
# robot's start pose, not the map origin. Without it a flawless odometry scores
# an ATE equal to the distance between the two frames — 12.0 m here, constant to
# the millimetre. AMCL above needs no alignment; it publishes in map already.
python3 analysis/localisation_error.py \
  --estimate "${RESULTS}/wheel_odom.csv" \
  --reference "${RESULTS}/ground_truth.csv" \
  --output "${RESULTS}/wheel_odom_error.json" \
  --align \
  --title "Wheel odometry against Gazebo ground truth (baseline)"

echo
echo "--- results written to ${RESULTS}/"
