#!/bin/bash

export PHA_HOME=/home/${USER}/schreibtisch/pha_docker_files
export SSI_PATH=${PHA_HOME}/docker_share
export ADDONS_PHA=${SSI_PATH}/git_pkgs/pha_addons
export SIMULATORS_PHA=${SSI_PATH}/git_pkgs/simulators

xhost +local:docker
