#!/bin/bash

export PHA_HOME=/home/${USER}/schreibtisch/pha_docker_files
export SSI_PATH=${PHA_HOME}/docker_share
export ADDONS_PHA=${SSI_PATH}/git_pkgs/pha_addons
export SIMULATORS_PHA=${SSI_PATH}/git_pkgs/simulators
export ROS_PHA=${SSI_PATH}/git_pkgs/ros_pkgs
source ${PHA_HOME}/docs/PHA_VERSION
export PHA_VERSION=$PHA_VERSION

alias pha="${PHA_HOME}/pha.sh"
source ${SSI_PATH}/scripts/setup/term_disp.sh

xhost +local:docker
