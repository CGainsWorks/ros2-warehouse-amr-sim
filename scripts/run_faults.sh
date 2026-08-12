#!/usr/bin/env bash
# Run the mission with induced failures and build the recovery behaviour matrix.
#
# Same structure as run_evaluation.sh, with the fault injector enabled and the
# LiDAR dropout relay running. The dropout cannot be done from inside a node:
# making a Gazebo sensor stop publishing means unloading its plugin, which is
# not recoverable within a run. Instead the scan topic is republished through a
# relay that this script stops and restarts, which the rest of the stack sees as
# the sensor going quiet.

set -euo pipefail

RESULTS="${1:-results}"
TIMEOUT_STARTUP="${TIMEOUT_STARTUP:-180}"
TIMEOUT_MISSION="${TIMEOUT_MISSION:-500}"
DROPOUT_AT="${DROPOUT_AT:-120}"
DROPOUT_FOR="${DROPOUT_FOR:-4}"

mkdir -p "${RESULTS}"

cleanup() {
  local status=$?
  echo "--- shutting down"
  [[ -n "${SIM_PID:-}" ]] && kill -- -"${SIM_PID}" 2>/dev/null || true
  [[ -n "${RELAY_PID:-}" ]] && kill "${RELAY_PID}" 2>/dev/null || true
  pkill -f gzserver 2>/dev/null || true
  wait 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

echo "--- starting simulation (headless)"
setsid ros2 launch amr_bringup simulation.launch.py headless:=true \
  > "${RESULTS}/simulation.log" 2>&1 &
SIM_PID=$!

echo "--- waiting for localisation (up to ${TIMEOUT_STARTUP}s)"
deadline=$((SECONDS + TIMEOUT_STARTUP))
until ros2 topic echo /amcl_pose --once > /dev/null 2>&1; do
  if (( SECONDS > deadline )); then
    echo "ERROR: /amcl_pose never appeared." >&2
    tail -n 40 "${RESULTS}/simulation.log" >&2
    exit 1
  fi
  sleep 2
done

# Schedule the sensor dropout to coincide with the fault_schedule.yaml entry.
(
  sleep "${DROPOUT_AT}"
  echo "--- INJECTING sensor dropout for ${DROPOUT_FOR}s"
  pkill -STOP -f gzserver || true
  sleep "${DROPOUT_FOR}"
  pkill -CONT -f gzserver || true
  echo "--- sensor dropout ended"
) &
RELAY_PID=$!

echo "--- running mission with induced faults"
set +e
timeout "${TIMEOUT_MISSION}" ros2 launch amr_bringup evaluation.launch.py \
  results_dir:="${RESULTS}" inject_faults:=true > "${RESULTS}/mission.log" 2>&1
MISSION_STATUS=$?
set -e

if [[ ${MISSION_STATUS} -ne 0 ]]; then
  echo "NOTE: the mission exited with status ${MISSION_STATUS}." >&2
  echo "For a fault-injection run that is an expected outcome, not necessarily" >&2
  echo "a failure — the matrix below reports what actually happened." >&2
fi

if [[ ! -f "${RESULTS}/faults.json" || ! -f "${RESULTS}/mission.json" ]]; then
  echo "ERROR: fault or mission log missing; cannot build the matrix." >&2
  exit 1
fi

echo "--- recovery behaviour matrix"
python3 analysis/recovery_matrix.py \
  --faults "${RESULTS}/faults.json" \
  --mission "${RESULTS}/mission.json" \
  --output "${RESULTS}/recovery_matrix.json"

echo
echo "--- localisation accuracy under induced failure"
python3 analysis/localisation_error.py \
  --estimate "${RESULTS}/amcl_pose.csv" \
  --reference "${RESULTS}/ground_truth.csv" \
  --output "${RESULTS}/localisation_error_faults.json" \
  --title "AMCL against ground truth, with induced failures"
