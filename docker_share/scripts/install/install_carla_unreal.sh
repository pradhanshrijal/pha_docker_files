#!/bin/bash

# source install_carla_unreal.sh /home/${USER}/docker_share/git_pkgs

# Variables
IN_CARLA_UNREAL_FOLDER=$1
IN_CARLA_UNREAL_FOLDER="${IN_CARLA_UNREAL_FOLDER:=/home/${USER}/docker_share/git_pkgs/simulators}"
cd ${IN_CARLA_UNREAL_FOLDER}

# Downloads
git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git UnrealEngine_4.26
git clone https://github.com/pradhanshrijal/carla -b feature/ubuntu-22

# Install Unity
cd UnrealEngine_4.26
./Setup.sh && ./GenerateProjectFiles.sh && make

# Install Carla
cd ..
cd carla
./Update.sh