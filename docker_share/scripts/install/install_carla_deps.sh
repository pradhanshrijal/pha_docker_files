#!/bin/bash

# Ex:
# source install_carla_deps.sh pha humble 0.9.15

# Install dependencies for carla

IN_USERNAME=$1
IN_ROS_VERSION=$2
IN_CARLA_VERSION=$3
IN_USERNAME="${IN_USERNAME:=pha}"
IN_ROS_VERSION="${IN_ROS_VERSION:=humble}"
IN_CARLA_VERSION="${IN_CARLA_VERSION:=0.9.15}"

sudo apt-get update
sudo apt-get install wget software-properties-common -y
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add
sudo apt-get install -y build-essential clang-12 lld-12 cmake ninja-build libvulkan1 libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git aria2 libstdc++-12-dev  libbz2-dev
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-12/bin/clang++ 180
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-12/bin/clang 180 

python3 -m pip install --user -Iv setuptools==47.3.1
python3 -m pip install --user distro wheel auditwheel
python3 -m pip install --upgrade setuptools
python3 -m pip install distro

sudo apt install aria2 -y

cd /home/${IN_USERNAME}/ros2_ws/src
git clone --recurse-submodules https://github.com/pradhanshrijal/ros-bridge -b feature/u22-${IN_CARLA_VERSION}
cd ..
sudo apt install ros-$IN_ROS_VERSION-derived-object-msgs -y
source /opt/ros/$IN_ROS_VERSION/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $IN_ROS_VERSION # $ROS_DISTRO
colcon build --symlink-install