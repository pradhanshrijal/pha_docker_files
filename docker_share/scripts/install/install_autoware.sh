#!/bin/bash

# Installation for Humble
# First input [$1] is the ROS 2 Folder - ex: /home/pha/ros2_ws

cd $1/src
git clone https://github.com/autowarefoundation/autoware -b $ROS_DISTRO

# Install Dependencies
## TensorRT: Complete the download manually
# wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/secure/8.5.3/local_repos/nv-tensorrt-local-repo-ubuntu2204-8.5.3-cuda-11.8_1.0-1_amd64.deb
# sudo dpkg -i nv-tensorrt-local-repo-ubuntu2204-8.5.3-cuda-11.8_1.0-1_amd64.deb
# sudo cp /var/nv-tensorrt-local-repo-ubuntu2204-8.5.3-cuda-11.8/nv-tensorrt-local-3E951519-keyring.gpg /usr/share/keyrings/
# sudo apt update
# sudo apt install tensorrt -y
# sudo apt install python3-libnvinfer-dev -y
# python3 -m pip install protobuf
# sudo apt install uff-converter-tf -y
# python3 -m pip install onnx
# sudo apt install onnx-graphsurgeon -y
# rm -rf nv-tensorrt-local-repo-ubuntu2204-8.5.3-cuda-11.8_1.0-1_amd64.deb

wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# For details: https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html
sudo apt update
rmw_implementation_dashed=$(eval sed -e "s/_/-/g" <<< "${rmw_implementation}")
sudo apt install ros-$ROS_DISTRO-${rmw_implementation_dashed} -y

# Taken from https://github.com/astuff/pacmod3#installation
sudo apt install apt-transport-https -y
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-$ROS_DISTRO-pacmod3 -y
sudo apt install ros-$ROS_DISTRO-cudnn-cmake-module -y


python3 -m pip install gdown

sudo apt install geographiclib-tools -y

# Add EGM2008 geoid grid to geographiclib
sudo geographiclib-get-geoids egm2008-1

clang_format_version=14.0.6
python3 -m pip install pre-commit clang-format==${clang_format_version}

sudo apt install golang -y

mkdir autoware_pkgs
vcs import autoware_pkgs < autoware/autoware.repos

rm -rf autoware

# git clone https://github.com/tier4/tensorrt_cmake_module

cd $1

source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
MAKEFLAGS=-j4 colcon build --parallel-workers=4 --symlink-install --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
source $1/install/setup.bash