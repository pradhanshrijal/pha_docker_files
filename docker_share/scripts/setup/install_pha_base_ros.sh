#!/bin/bash

# Installation of base ROS 2 packages for the Pha Project
# First input [$1] is the ROS 2 Folder - ex: /home/pha/ros2_ws

cd $1/src
mkdir git_packages
mkdir perception
mkdir navigation
mkdir localization
mkdir drivers

cd navigation
git clone https://github.com/ros-planning/navigation2 -b $ROS_DISTRO

cd ../localization
git clone https://github.com/cra-ros-pkg/robot_localization -b ros2

cd ../perception
git clone https://github.com/ros-perception/perception_pcl -b ros2

cd ../drivers
git clone https://github.com/IntelRealSense/realsense-ros -b ros2-develoment
cd $1
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
MAKEFLAGS=-j4 colcon build --parallel-workers=4 --symlink-install --event-handlers --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash