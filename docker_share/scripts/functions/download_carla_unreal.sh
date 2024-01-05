#!/bin/bash

# Make sure your unity account is connected to your github. 
# This setup is for running it from outside docker.
P_INSTALL_FOLDER=/home/${USER}/schreibtisch/pha_docker_files/docker_share/git_pkgs/simulators

cd $P_INSTALL_FOLDER
git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git UnrealEngine_4.26
git clone https://github.com/pradhanshrijal/carla -b feature/ubuntu-22