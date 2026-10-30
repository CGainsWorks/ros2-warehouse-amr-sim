#!/usr/bin/env bash
# The one-command demo referenced by the README.
#
# Prefers Docker, because that pins gz-sim Harmonic against Jazzy, and falls
# back to a native workspace if one is already sourced. If neither is available
# it says so and points at the offline checks, which need nothing but Python and
# reproduce most of the repository's quantitative claims.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

MODE="${1:-auto}"

run_docker() {
  echo "--- running the demo in Docker"
  docker compose -f docker/docker-compose.yml up --build gui
}

run_native() {
  echo "--- running the demo natively"
  # shellcheck disable=SC1091
  source install/setup.bash
  ros2 launch amr_bringup simulation.launch.py headless:=false use_rviz:=true
}

case "${MODE}" in
  docker) run_docker ;;
  native) run_native ;;
  auto)
    if [[ -f install/setup.bash ]]; then
      run_native
    elif command -v docker > /dev/null 2>&1; then
      run_docker
    else
      cat >&2 <<'EOF'
Neither a built workspace nor Docker was found.

Build the workspace with:      ./scripts/setup.sh
Or install Docker and run:     make docker-demo

The offline metrics need neither, and reproduce the segmentation and
scan-matching numbers quoted in the README in a few seconds:

    python3 -m pip install numpy scipy pytest
    make check
EOF
      exit 1
    fi
    ;;
  *)
    echo "usage: $0 [auto|docker|native]" >&2
    exit 2
    ;;
esac
