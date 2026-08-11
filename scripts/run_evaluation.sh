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
TIMEOUT_MISSION="${TIMEOUT_MISSION:-400}"

mkdir -p "${RESULTS}"

cleanup() {
  local status=$?
  echo "--- shutting down"
  # Kill the whole process group: ros2 launch spawns children that do not
  # always die with their parent, and a leftover gzserver holds port 11345 and
  # makes the next run fail with a misleading error.
  [[ -n "${SIM_PID:-}" ]] && kill -- -"${SIM_PID}" 2>/dev/null || true
  pkill -f gzserver 2>/dev/null || true
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

echo "--- starting simulation (headless)"
setsid ros2 launch amr_bringup simulation.launch.py headless:=true \
  > "${RESULTS}/simulation.log" 2>&1 &
SIM_PID=$!

echo "--- waiting for localisation to come up (up to ${TIMEOUT_STARTUP}s)"
deadline=$((SECONDS + TIMEOUT_STARTUP))
until ros2 topic echo /amcl_pose --once > /dev/null 2>&1; do
  if (( SECONDS > deadline )); then
    echo "ERROR: /amcl_pose never appeared. Last 40 lines of the simulation log:" >&2
    tail -n 40 "${RESULTS}/simulation.log" >&2
    exit 1
  fi
  sleep 2
done
echo "--- localisation is up"

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

echo "--- localisation accuracy"
python3 analysis/localisation_error.py \
  --estimate "${RESULTS}/amcl_pose.csv" \
  --reference "${RESULTS}/ground_truth.csv" \
  --output "${RESULTS}/localisation_error.json" \
  --title "AMCL against Gazebo ground truth"

echo
python3 analysis/localisation_error.py \
  --estimate "${RESULTS}/wheel_odom.csv" \
  --reference "${RESULTS}/ground_truth.csv" \
  --output "${RESULTS}/wheel_odom_error.json" \
  --title "Wheel odometry against Gazebo ground truth (baseline)"

echo
echo "--- results written to ${RESULTS}/"
