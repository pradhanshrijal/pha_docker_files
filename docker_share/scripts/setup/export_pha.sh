#!/bin/bash

export PHA_HOME=/home/${USER}/schreibtisch/pha_docker_files
export SSI_PATH=${PHA_HOME}/docker_share
export ADDONS_PHA=${SSI_PATH}/git_pkgs/pha_addons
export SIMULATORS_PHA=${SSI_PATH}/git_pkgs/simulators
export ROS_PKGS_PHA=${SSI_PATH}/git_pkgs/ros_pkgs
alias pha="${PHA_HOME}/pha.sh"

xhost +local:docker
