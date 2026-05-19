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

if [ ! -d local_files ]; then
  mkdir local_files
fi

cd local_files

# Install Requirements
if [ "$SUDO_ACC" = "true" ]; then
  sudo apt-get install git git-lfs python3 python3-pip x11-xserver-utils -y
  python3 -m pip install gdown==4.6.1
else
  python3 -m venv venv
  source venv/bin/activate
  echo "source ${PHA_PARENT}/local_files/venv/bin/activate" >> /home/${USER}/.bashrc
  python3 -m pip install gdown==4.6.1
fi
#

cd ${PHA_PARENT}/pha_docker_files

if [[ -z "${PHA_HOME}" ]]; then
  echo -e "\n# PHA" >> /home/${USER}/.bashrc
  echo "source ${PHA_PARENT}/pha_docker_files/docker_share/scripts/setup/export_pha.sh" >> /home/${USER}/.bashrc
  source /home/${USER}/.bashrc
fi
#