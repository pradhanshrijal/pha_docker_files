#!/bin/bash

export CONDA_INSTALL_VERSION=3-2022.05
export CONDA_INSTALL_FILE=Anaconda$CONDA_INSTALL_VERSION-Linux-x86_64
wget https://repo.anaconda.com/archive/$CONDA_INSTALL_FILE.sh
bash $CONDA_INSTALL_FILE.sh -b -p conda
rm -rf $CONDA_INSTALL_FILE.sh
