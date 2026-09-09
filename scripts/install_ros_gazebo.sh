#!/usr/bin/env bash
set -euo pipefail

# 1. Ensure system locale supports UTF-8
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Install preliminary system dependencies
sudo apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    software-properties-common

# 3. Enable Ubuntu Universe repository
sudo add-apt-repository -y universe

# 4. Add official ROS 2 Apt Repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. Add OSRF Gazebo Repository
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 6. Update package lists and install ROS 2 Humble Base + Gazebo Harmonic dependencies
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-pytest \
    build-essential \
    xterm

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "ROS 2 Humble and Gazebo Harmonic installation complete!"