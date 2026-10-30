#!/usr/bin/env bash
# Source ROS and the built workspace, then hand off.
#
# `set -e` is deliberately NOT combined with sourcing the setup scripts before
# the guard below: ROS's setup.bash trips over unset variables under `set -u`,
# so the sourcing happens first and strict mode is enabled afterwards.

source /opt/ros/jazzy/setup.bash
if [ -f "${WORKSPACE:-/ws}/install/setup.bash" ]; then
  source "${WORKSPACE:-/ws}/install/setup.bash"
fi

set -euo pipefail

# gz-sim writes logs and its render cache under ~/.gz and fails opaquely if
# HOME is unwritable, which happens when the container is run with --user.
export HOME="${HOME:-/tmp}"
mkdir -p "${HOME}/.gz"

exec "$@"
