#!/bin/bash

# Ex:
# source install_carla_bridge_deps.sh pha humble 0.9.15

# Install dependencies for carla

IN_USERNAME=$1
IN_ROS_VERSION=$2
IN_CARLA_VERSION=$3
IN_USERNAME="${IN_USERNAME:=pha}"
IN_ROS_VERSION="${IN_ROS_VERSION:=humble}"
IN_CARLA_VERSION="${IN_CARLA_VERSION:=0.9.15}"

# Install Dependencies
python3 -m pip install carla==${IN_CARLA_VERSION}
python3 -m pip install pygame
sudo apt install ros-${IN_ROS_VERSION}-derived-object-msgs -y
python3 -m pip install numpy==1.23.1

# ROS Install
cd /home/${IN_USERNAME}/docker_share/git_pkgs
mkdir -p test_ws/src
cd test_ws/src
git clone --recurse-submodules https://github.com/pradhanshrijal/ros-bridge -b feature/u22-${IN_CARLA_VERSION}
cd ..
source /opt/ros/${IN_ROS_VERSION}/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro ${IN_ROS_VERSION} # $ROS_DISTRO
colcon build --symlink-install