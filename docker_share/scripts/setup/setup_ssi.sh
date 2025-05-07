#!/bin/bash

# Setup Carlaware
# Sample: source setup_cwr.sh /home/pha/schreibtisch

# Variables
SUDO_ACC=$1
SUDO_ACC="${SUDO_ACC:=true}"
PHA_PARENT=$2
PHA_PARENT="${PHA_PARENT:=/home/${USER}/schreibtisch}"
PHA_DB=$3
PHA_DB="${PHA_DB:=github.com}"
#

# Install Requirements
if [ "$SUDO_ACC" = "true" ]; then
  sudo apt-get install git git-lfs python3 python3-pip x11-xserver-utils -y
  python3 -m pip install gdown==4.6.1
fi
#

# Parent Folder
if [ ! -d "$PHA_PARENT" ]; then
  mkdir -p $PHA_PARENT
fi
#

# PHA Folder
cd $PHA_PARENT

if [ ! -d pha_docker_files ]; then
  git clone https://${PHA_DB}/pradhanshrijal/pha_docker_files --recursive
fi

cd pha_docker_files

if [[ -z "${PHA_HOME}" ]]; then
  echo -e "\n# PHA" >> /home/${USER}/.bashrc
  echo "source ${PHA_PARENT}/pha_docker_files/docker_share/scripts/setup/export_pha.sh" >> /home/${USER}/.bashrc
  source /home/${USER}/.bashrc
fi
#