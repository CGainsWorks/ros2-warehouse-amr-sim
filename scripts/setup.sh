#!/usr/bin/env bash
# Set up a native ROS 2 Humble workspace on Ubuntu 22.04.
#
# Docker is the supported path — see docker/docker-compose.yml — because it
# pins Gazebo Classic 11 against Humble and that pairing is the fragile part.
# This script exists for working on a machine that already has Humble.
#
# It is idempotent: re-running it is safe.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  cat >&2 <<'EOF'
ERROR: ROS 2 Humble was not found at /opt/ros/humble.

This stack targets Humble specifically, because it uses Gazebo Classic 11
plugins (gazebo_ros_diff_drive, gazebo_ros_ray_sensor, gazebo_ros_p3d) whose
Ignition equivalents have different names and parameters.

Either install Humble, or use the container, which needs neither:

    make docker-check    # offline metrics, no simulator
    make docker-demo     # the full simulation
EOF
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

echo "--- installing dependencies"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  python3-pip python3-numpy python3-scipy python3-yaml \
  python3-colcon-common-extensions python3-rosdep \
  gazebo libgazebo-dev \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-cartographer ros-humble-cartographer-ros \
  ros-humble-xacro ros-humble-rviz2

pip3 install --user --no-cache-dir pytest flake8

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble
rosdep install --from-paths src --ignore-src -y --rosdistro humble \
  --skip-keys "gazebo libgazebo-dev"

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
