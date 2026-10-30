#!/usr/bin/env bash
# Set up a native ROS 2 Jazzy workspace on Ubuntu 24.04.
#
# Docker is the supported path (see docker/docker-compose.yml) because it pins
# gz-sim Harmonic against Jazzy. This script exists for working on a machine
# that already has Jazzy. Jazzy's ros-jazzy-ros-gz packages bring Harmonic
# with them through the gz_*_vendor packages, so there is no separate Gazebo
# install to get wrong.
#
# It is idempotent: re-running it is safe.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  cat >&2 <<'EOF'
ERROR: ROS 2 Jazzy was not found at /opt/ros/jazzy.

This stack targets Jazzy with gz-sim Harmonic, the officially paired
combination; see docs/simulation.md for why.

Either install Jazzy, or use the container, which needs neither:

    make docker-check    # offline metrics, no simulator
    make docker-demo     # the full simulation
EOF
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

echo "--- installing dependencies"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  python3-numpy python3-scipy python3-yaml python3-pytest python3-paho-mqtt flake8 \
  python3-colcon-common-extensions python3-rosdep mosquitto \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-cartographer ros-jazzy-cartographer-ros \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-launch-testing ros-jazzy-launch-testing-ros \
  ros-jazzy-xacro ros-jazzy-rviz2

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy

echo "--- regenerating the world, zones and reference map"
python3 simulation/generate_world.py

echo "--- building"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "--- verifying (offline checks, no simulator)"
python3 -m pytest tests/ -q

cat <<EOF

Done. Source the workspace and run the demo:

    source install/setup.bash
    make demo

Or reproduce the offline metrics without the simulator:

    make check
EOF
