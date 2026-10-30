#!/usr/bin/env bash
# Exercise the recovery behaviours, one induced failure per run, then build the
# recovery behaviour matrix from all of them.
#
# ONE FAULT PER RUN, deliberately. Firing the whole schedule into a single
# mission looks efficient and produces numbers that cannot be read:
#
#   * A fault that ends the run hides every fault scheduled after it. The
#     sensor dropout sat at 120 s behind a kidnap the stack is not designed to
#     survive, and had therefore never once executed.
#   * A fault lands on a robot the previous fault already broke. Measured in
#     one combined run, the dropout at 55 s was applied to a filter that the
#     pose jump at 35 s had left 8 m out, so the "recovery latency" attributed
#     to it was largely the previous fault still unwinding.
#
# This is the same decoupling argument as the open-loop localisation harness in
# run_openloop.sh: to measure a thing, stop the other things from feeding into
# it. Five short runs cost more wall-clock than one long one and are the only
# version that answers the question.
#
# The LiDAR dropout needs the relay: a Gazebo sensor cannot be stopped and
# restarted inside a run, since making it stop publishing means unloading the
# plugin. `use_scan_relay:=true` moves the LiDAR onto scan_raw and interposes
# amr_bringup's scan_relay, which the injector tells to stop forwarding.
#
# The relay is in the path *only* here. Every other run wires the sensor
# straight to the stack, which is how the figures in the docs were measured.

set -euo pipefail

RESULTS="${1:-results}"
TIMEOUT_STARTUP="${TIMEOUT_STARTUP:-180}"
# Wall-clock, and the simulation runs at roughly 0.6x real time on the machine
# this was developed on, so a 160 s mission takes about 260 s to watch. 300 was
# not enough: the sensor-dropout run survived its fault, kept driving, and was
# killed mid-mission, which the matrix then reported as "did not reach the
# fault" — the opposite of what happened.
TIMEOUT_MISSION="${TIMEOUT_MISSION:-600}"
SCHEDULE="${SCHEDULE:-$(ros2 pkg prefix amr_bringup)/share/amr_bringup/config/fault_schedule.yaml}"
# Every fault fires at the same mission time, so each one meets a robot in the
# same state. Late enough to be mid-leg, early enough to leave room to recover.
FAULT_AT="${FAULT_AT:-25.0}"

FAULT_ROOT="${RESULTS}/faults"
mkdir -p "${FAULT_ROOT}"

SIM_PID=""

# Between runs, the gz sim server has to be *gone*, not merely signalled. The first
# version slept three seconds and hoped. Run 2 then died with
#   Failed init_port fastrtps_port17919: open_and_lock_file failed
# because run 1's DDS shared-memory segments were still held, and since the
# readiness check below could not time out, the harness sat there for eighteen
# minutes with nothing to show for it.
cleanup_sim() {
  [[ -n "${SIM_PID}" ]] && kill -- -"${SIM_PID}" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  SIM_PID=""

  local waited=0
  while pgrep -f "gz sim" > /dev/null 2>&1; do
    if (( waited >= 20 )); then
      echo "    gz sim will not exit; SIGKILL"
      pkill -9 -f "gz sim" 2>/dev/null || true
      sleep 2
      break
    fi
    sleep 1
    waited=$((waited + 1))
  done

  # Stale DDS segments outlive the process that made them and stop the next
  # run from binding. Nothing else in this container uses them.
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  sleep 3
}
trap 'echo "--- shutting down"; cleanup_sim; exit' EXIT INT TERM

mapfile -t FAULT_TYPES < <(python3 -c "
import sys, yaml
for fault in yaml.safe_load(open('${SCHEDULE}'))['faults']:
    print(fault['type'])
")

# ONLY=block_aisle,sensor_dropout reruns a subset. Legitimate because the runs
# are independent by construction — that is the whole point of one per run.
ONLY="${ONLY:-}"

echo "--- ${#FAULT_TYPES[@]} faults, one run each: ${FAULT_TYPES[*]}"
[[ -n "${ONLY}" ]] && echo "--- restricted to: ${ONLY}"

for index in "${!FAULT_TYPES[@]}"; do
  kind="${FAULT_TYPES[$index]}"
  if [[ -n "${ONLY}" && ",${ONLY}," != *",${kind},"* ]]; then
    echo "    skipping ${kind} (ONLY=${ONLY})"
    continue
  fi
  run_dir="${FAULT_ROOT}/${kind}"
  mkdir -p "${run_dir}"

  # A schedule holding exactly one fault, retimed to FAULT_AT.
  python3 -c "
import yaml
schedule = yaml.safe_load(open('${SCHEDULE}'))
fault = schedule['faults'][${index}]
fault['at'] = ${FAULT_AT}
yaml.safe_dump({'faults': [fault]}, open('${run_dir}/schedule.yaml', 'w'))
"

  echo
  echo "=== [$((index + 1))/${#FAULT_TYPES[@]}] ${kind} at t=${FAULT_AT}s"

  setsid ros2 launch amr_bringup simulation.launch.py headless:=true \
    use_scan_relay:=true > "${run_dir}/simulation.log" 2>&1 &
  SIM_PID=$!

  # `timeout` is not optional here: `ros2 topic echo --once` blocks forever on a
  # topic that never appears, so without it the deadline below is unreachable.
  deadline=$((SECONDS + TIMEOUT_STARTUP))
  until timeout 5 ros2 topic echo /amcl_pose --once > /dev/null 2>&1; do
    if (( SECONDS > deadline )); then
      echo "    ERROR: /amcl_pose never appeared for ${kind}; skipping this fault." >&2
      grep -iE "process has died|Spawn service failed|init_port"         "${run_dir}/simulation.log" | head -n 5 >&2 || true
      cleanup_sim
      continue 2
    fi
    sleep 2
  done

  set +e
  timeout "${TIMEOUT_MISSION}" ros2 launch amr_bringup evaluation.launch.py \
    results_dir:="${run_dir}" inject_faults:=true \
    fault_schedule:="${run_dir}/schedule.yaml" > "${run_dir}/mission.log" 2>&1
  status=$?
  set -e

  if [[ ${status} -ne 0 ]]; then
    echo "    mission exited ${status} — expected for a fault run, not a failure"
  fi

  cleanup_sim
done

echo
echo "--- recovery behaviour matrix"
python3 analysis/recovery_matrix.py \
  --runs "${FAULT_ROOT}" \
  --output "${RESULTS}/recovery_matrix.json"
