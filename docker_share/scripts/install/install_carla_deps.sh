#!/bin/bash

# Install dependencies for carla

IN_USERNAME=$1
IN_ROS_VERSION=$2
IN_USERNAME="${IN_USERNAME:=pha}"
IN_ROS_VERSION="${IN_ROS_VERSION:=humble}"
IN_CARLA_ROOT=/home/${IN_USERNAME}/docker_share/git_pkgs/simulators/carla
IN_UE4_ROOT=/home/${IN_USERNAME}/docker_share/git_pkgs/simulators/UnrealEngine_4.26
source /home/${IN_USERNAME}/.bashrc

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
git clone --recurse-submodules https://github.com/gezp/carla_ros -b humble-carla-0.9.14
cd ..
sudo apt install ros-$IN_ROS_VERSION-derived-object-msgs -y
source /opt/ros/$IN_ROS_VERSION/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $IN_ROS_VERSION # $ROS_DISTRO
colcon build --symlink-install

echo "# Unreal | Carla"
echo "export UE4_ROOT=${IN_UE4_ROOT}" >> /home/${IN_USERNAME}/.bashrc
echo "export CARLA_ROOT=${IN_CARLA_ROOT}" >> /home/${IN_USERNAME}/.bashrc
echo "export PYTHONPATH=\$PYTHONPATH:${IN_CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg:${IN_CARLA_ROOT}/PythonAPI/carla" >> /home/${IN_USERNAME}/.bashrc
source /home/${IN_USERNAME}/.bashrc