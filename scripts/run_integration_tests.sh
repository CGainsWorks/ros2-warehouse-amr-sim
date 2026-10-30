#!/usr/bin/env bash
# The two headless integration tests, in sequence, from a built and sourced
# workspace. This is what CI runs and what `docker compose run integration`
# runs. Each brings up its own simulation, so they cannot share state.
#
#   navigation_recovery  one mission, one induced failure, recovery, success
#   vda5050_order        an order over MQTT, driven to completion
#
# Summaries land in $1 (default results/integration) as JSON. Every number in
# them is simulated.

set -uo pipefail

RESULTS="${1:-results/integration}"
mkdir -p "${RESULTS}"
export AMR_TEST_RESULTS
AMR_TEST_RESULTS="$(cd "${RESULTS}" && pwd)"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
status=0
for test in src/amr_bringup/test/test_navigation_recovery.launch.py \
            src/amr_fleet/test/test_vda5050_order.launch.py; do
  name="$(basename "${test}" .launch.py)"
  echo "=== ${name}"
  # launch_test exits non-zero on any failed assertion, in either the active
  # or the post-shutdown tests. The log is kept whatever happens.
  if launch_test "${REPO_ROOT}/${test}" > "${RESULTS}/${name}.log" 2>&1; then
    echo "    passed"
  else
    status=1
    echo "    FAILED; the last 60 lines of ${RESULTS}/${name}.log:"
    tail -n 60 "${RESULTS}/${name}.log"
  fi
  grep -aE '^(test_|Ran |OK|FAILED)|\.\.\. (ok|FAIL|ERROR)$' "${RESULTS}/${name}.log" || true
  # A gz server can outlive its launch (ros_gz_sim runs it under a ruby
  # wrapper). Each test has its own gz partition, but a leftover server
  # would still take CPU from the next test, so it goes.
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -x mosquitto 2>/dev/null || true
  sleep 2
done
exit "${status}"
