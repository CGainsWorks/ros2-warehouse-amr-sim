# ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8) + Nav2 + Cartographer.
#
# Jazzy and Harmonic are the officially paired combination (REP 2000): both are
# long-term releases, Jazzy is supported to May 2029 and Harmonic to September
# 2028, and Jazzy's ros_gz packages are built against Harmonic through the
# gz_*_vendor packages, so `apt install ros-jazzy-ros-gz` brings the matching
# simulator with it and nothing has to be pinned by hand. Humble's official
# partner, Fortress, reached end of life in September 2026, and Gazebo
# Classic 11, which this stack used before, in January 2025. See
# docs/simulation.md for the full reasoning.

FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# System and ROS dependencies. One layer, cleaned in the same layer so the
# apt lists do not end up in the image. Ubuntu 24.04 marks the system Python
# as externally managed, so the Python tools come from apt rather than pip.
#
#   mosquitto          the MQTT broker the VDA 5050 integration test starts
#   python3-paho-mqtt  the VDA 5050 adapter's MQTT client (1.6 on noble)
#   ros-jazzy-tf2, ros-jazzy-tf2-ros
#                      listed so apt upgrades them in an older base image.
#                      tf2 0.36.22 has an ABBA deadlock between
#                      waitForTransform and testTransformableRequests (fixed
#                      in 0.36.23). It froze AMCL and both costmaps a minute
#                      or two into a run; see docs/simulation.md.
#   ros-jazzy-rmw-cyclonedds-cpp  the middleware the stack runs on
RUN apt-get update && apt-get install -y --no-install-recommends \
      make \
      git \
      python3-numpy \
      python3-scipy \
      python3-yaml \
      python3-pytest \
      python3-paho-mqtt \
      flake8 \
      mosquitto \
      python3-colcon-common-extensions \
      ros-jazzy-ros-gz \
      ros-jazzy-tf2 \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-robot-localization \
      ros-jazzy-cartographer \
      ros-jazzy-cartographer-ros \
      ros-jazzy-tf2-ros \
      ros-jazzy-tf2-sensor-msgs \
      ros-jazzy-rviz2 \
      ros-jazzy-launch-testing \
      ros-jazzy-launch-testing-ros \
      ros-jazzy-launch-testing-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Fail the build rather than ship the deadlocking tf2.
RUN dpkg --compare-versions "$(dpkg-query -W -f='${Version}' ros-jazzy-tf2)" ge 0.36.23

# Cyclone DDS, and discovery kept on the container's own loopback; see
# docs/simulation.md for why each matters.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    GZ_IP=127.0.0.1

ENV WORKSPACE=/ws
WORKDIR ${WORKSPACE}

# Copy the manifests first so that a source-only change does not invalidate the
# rosdep layer.
COPY src/amr_common/package.xml       src/amr_common/package.xml
COPY src/amr_msgs/package.xml         src/amr_msgs/package.xml
COPY src/amr_description/package.xml  src/amr_description/package.xml
COPY src/amr_localisation/package.xml src/amr_localisation/package.xml
COPY src/amr_navigation/package.xml   src/amr_navigation/package.xml
COPY src/amr_perception/package.xml   src/amr_perception/package.xml
COPY src/amr_fleet/package.xml        src/amr_fleet/package.xml
COPY src/amr_bringup/package.xml      src/amr_bringup/package.xml

RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

COPY . ${WORKSPACE}

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
# The scripts are mode 100755 in git, but a checkout made on Windows can arrive
# without the bit, and the compose services invoke them as ./scripts/foo.sh.
# Setting it here costs nothing and turns a confusing "Permission denied" at
# `docker compose run eval` into a non-event.
RUN chmod +x /entrypoint.sh ${WORKSPACE}/scripts/*.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "amr_bringup", "simulation.launch.py", "headless:=true"]
