# Copyright 2026 amr-sim-lab contributors
# Licensed under the Apache License, Version 2.0. See LICENSE.
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# Gazebo Harmonic via the OSRF apt repo + the ros_gz build against it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        ros-humble-ros-gzharmonic-sim \
        ros-humble-ros-gz-sim \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions \
        python3-pytest \
        build-essential \
        xterm \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR /ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
