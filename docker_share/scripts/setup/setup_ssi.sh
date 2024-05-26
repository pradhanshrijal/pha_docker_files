#!/bin/bash

PHA_PARENT=/home/${USER}/schreibtisch

if [ ! -d "$PHA_PARENT" ]; then
  mkdir -p $PHA_PARENT
fi

cd $PHA_PARENT

if [ ! -d pha_docker_files ]; then
  git clone https://github.com/pradhanshrijal/pha_docker_files --recursive
fi

cd pha_docker_files

if [[ -z "${PHA_HOME}" ]]; then
    echo "# PHA" >> /home/${USER}/.bashrc
    echo "source ${PHA_PARENT}/pha_docker_files/docker_share/scripts/setup/export_pha.sh" >> /home/${USER}/.bashrc
fi