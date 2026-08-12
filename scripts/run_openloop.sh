#!/usr/bin/env bash
# Measure localisation quality with the navigation stack out of the loop.
#
# A Nav2 mission cannot be used to tune localisation, because the two are
# coupled: a localisation error makes the costmap wrong, Nav2 then fails to
# progress and runs recovery spins, the spins slip the wheels, and the slip
# corrupts the odometry that localisation depends on. Every number moves for
# reasons unrelated to whatever was being tuned.
#
# This drives a fixed velocity profile instead. Nothing reads any estimate, so
# the trajectory is identical run to run however badly localisation performs,
# and the resulting ATE is a measurement of the estimator alone.
#
#   USE_SCAN_ODOMETRY=false ./scripts/run_openloop.sh results/openloop_noscan
#
set -euo pipefail

RESULTS="${1:-results/openloop}"
TIMEOUT_STARTUP="${TIMEOUT_STARTUP:-180}"
TIMEOUT_PROFILE="${TIMEOUT_PROFILE:-300}"
USE_SCAN_ODOMETRY="${USE_SCAN_ODOMETRY:-false}"

mkdir -p "${RESULTS}"

cleanup() {
  local status=$?
  echo "--- shutting down"
  [[ -n "${SIM_PID:-}" ]] && kill -- -"${SIM_PID}" 2>/dev/null || true
  pkill -f gzserver 2>/dev/null || true
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

echo "--- starting simulation without Nav2 (scan odometry: ${USE_SCAN_ODOMETRY})"
setsid ros2 launch amr_bringup simulation.launch.py \
  headless:=true use_navigation:=false \
  use_scan_odometry:="${USE_SCAN_ODOMETRY}" \
  > "${RESULTS}/simulation.log" 2>&1 &
SIM_PID=$!

echo "--- waiting for localisation (up to ${TIMEOUT_STARTUP}s)"
deadline=$((SECONDS + TIMEOUT_STARTUP))
# timeout, so the deadline below is reachable: `ros2 topic echo --once` blocks
# forever on a topic that never appears.
until timeout 5 ros2 topic echo /amcl_pose --once > /dev/null 2>&1; do
  if (( SECONDS > deadline )); then
    echo "ERROR: /amcl_pose never appeared." >&2
    tail -n 30 "${RESULTS}/simulation.log" >&2
    exit 1
  fi
  sleep 3
done
echo "--- localisation up after ${SECONDS}s"

echo "--- logging poses and driving the profile"
setsid ros2 run amr_bringup pose_logger --ros-args \
  -p use_sim_time:=true -p output_dir:="${RESULTS}" \
  > "${RESULTS}/pose_logger.log" 2>&1 &
LOGGER_PID=$!
sleep 3

set +e
timeout "${TIMEOUT_PROFILE}" ros2 run amr_bringup openloop_driver --ros-args \
  -p use_sim_time:=true > "${RESULTS}/driver.log" 2>&1
DRIVER_STATUS=$?
set -e
echo "--- profile finished (status ${DRIVER_STATUS})"

# SIGINT so the logger flushes and closes its CSVs cleanly.
kill -INT -- -"${LOGGER_PID}" 2>/dev/null || true
sleep 3

echo
echo "--- odometry audit"
python3 analysis/odometry_audit.py --results "${RESULTS}" \
  --output "${RESULTS}/odometry_audit.json"

echo
python3 analysis/localisation_error.py \
  --estimate "${RESULTS}/amcl_pose.csv" \
  --reference "${RESULTS}/ground_truth.csv" \
  --output "${RESULTS}/localisation_error.json" \
  --title "AMCL against ground truth (open loop)"

echo
python3 analysis/localisation_error.py \
  --estimate "${RESULTS}/ekf_odom.csv" \
  --reference "${RESULTS}/ground_truth.csv" \
  --output "${RESULTS}/ekf_error.json" \
  --align \
  --title "EKF against ground truth (open loop)"

echo
python3 analysis/localisation_error.py \
  --estimate "${RESULTS}/wheel_odom.csv" \
  --reference "${RESULTS}/ground_truth.csv" \
  --output "${RESULTS}/wheel_odom_error.json" \
  --align \
  --title "Wheel odometry against ground truth (open loop)"

echo
echo "--- results in ${RESULTS}/"
