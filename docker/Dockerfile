# ROS 2 Humble + Gazebo Classic 11 + Nav2 + Cartographer.
#
# Humble pairs with Gazebo Classic 11, not with Ignition/Gazebo Sim. The
# gazebo_ros_pkgs plugins this stack depends on — diff drive, ray sensor, p3d
# ground truth — are Classic plugins, and the Ignition equivalents have
# different names and parameters. Pinning the base image to humble rather than
# rolling is what keeps that pairing stable.

FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# System and ROS dependencies. One layer, cleaned in the same layer so the
# apt lists do not end up in the image.
RUN apt-get update && apt-get install -y --no-install-recommends \
      make \
      git \
      python3-pip \
      python3-numpy \
      python3-scipy \
      python3-yaml \
      python3-colcon-common-extensions \
      gazebo \
      libgazebo-dev \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros \
      ros-humble-xacro \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-robot-localization \
      ros-humble-cartographer \
      ros-humble-cartographer-ros \
      ros-humble-tf2-ros \
      ros-humble-tf2-sensor-msgs \
      ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pytest flake8

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
COPY src/amr_bringup/package.xml      src/amr_bringup/package.xml

RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
       --skip-keys "gazebo libgazebo-dev" \
    && rm -rf /var/lib/apt/lists/*

COPY . ${WORKSPACE}

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Gazebo needs to find the generated world's model references.
ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${WORKSPACE}/simulation/models
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:${WORKSPACE}/simulation

COPY docker/entrypoint.sh /entrypoint.sh
# The scripts are mode 100755 in git, but a checkout made on Windows can arrive
# without the bit, and the compose services invoke them as ./scripts/foo.sh.
# Setting it here costs nothing and turns a confusing "Permission denied" at
# `docker compose run eval` into a non-event.
RUN chmod +x /entrypoint.sh ${WORKSPACE}/scripts/*.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "amr_bringup", "simulation.launch.py", "headless:=true"]
