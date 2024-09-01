#!/bin/bash

# Setup Carlaware
# Sample: source setup_cwr.sh /home/pha/schreibtisch

# Variables
PHA_PARENT=$1
PHA_PARENT="${PHA_PARENT:=/home/${USER}/schreibtisch}"
PHA_URL=$2
PHA_URL="${PHA_URL:=https://github.com/pradhanshrijal/pha_docker_files}"
#

# Install Requirements
sudo apt-get install python3 python3-pip -y
python3 -m pip install gdown
#

# Parent Folder
if [ ! -d "$PHA_PARENT" ]; then
  mkdir -p $PHA_PARENT
fi
#

# PHA Folder
cd $PHA_PARENT

if [ ! -d pha_docker_files ]; then
  git clone ${PHA_URL} --recursive
fi

cd pha_docker_files

if [[ -z "${PHA_HOME}" ]]; then
  echo "# PHA" >> /home/${USER}/.bashrc
  echo "source ${PHA_PARENT}/pha_docker_files/docker_share/scripts/setup/export_pha.sh" >> /home/${USER}/.bashrc
  source /home/${USER}/.bashrc
fi
#