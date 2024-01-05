#!/bin/bash

# source install_carla_unreal.sh /home/${USER}/docker_share/git_pkgs/simulators

# Variables
IN_CARLA_UNREAL_FOLDER=$1
IN_CARLA_VERSION=$2
IN_CARLA_UNREAL_FOLDER="${IN_CARLA_UNREAL_FOLDER:=/home/${USER}/docker_share/git_pkgs/simulators}"
IN_CARLA_VERSION="${IN_CARLA_VERSION:=0.9.15}"
IN_CARLA_ROOT=${IN_CARLA_UNREAL_FOLDER}/carla
IN_UE4_ROOT=${IN_CARLA_UNREAL_FOLDER}/UnrealEngine_4.26
source /home/${USER}/.bashrc
cd ${IN_CARLA_UNREAL_FOLDER}

# Downloads
git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git UnrealEngine_4.26
git clone https://github.com/pradhanshrijal/carla -b feature/u22-${IN_CARLA_VERSION}

# Install Unity
cd UnrealEngine_4.26
./Setup.sh && ./GenerateProjectFiles.sh && make

echo -e "\n# Unreal | Carla"
echo "export UE4_ROOT=${IN_UE4_ROOT}" >> /home/${USER}/.bashrc
echo "export CARLA_ROOT=${IN_CARLA_ROOT}" >> /home/${USER}/.bashrc
echo "export PYTHONPATH=\$PYTHONPATH:${IN_CARLA_ROOT}/PythonAPI/carla/dist/carla-${IN_CARLA_VERSION}-py3.10-linux-x86_64.egg:${IN_CARLA_ROOT}/PythonAPI/carla" >> /home/${USER}/.bashrc
source /home/${USER}/.bashrc

# Install Carla
cd ..
cd carla
./Update.sh