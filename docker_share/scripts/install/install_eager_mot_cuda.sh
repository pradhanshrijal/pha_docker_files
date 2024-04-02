#!/bin/bash

# File Tested with CUDA 10.0 in Ubuntu 18.04
# Input is the name of the Conda Container - ex: eager_conda

# To Change variables and environments, find all instance in the file and repalce them
export EXT_DATA_PATH=/media/$USER/storage_drive
export SCRIPTS_FOLDER=/home/$USER/docker_share/setup
export SOFTWARE_PARENT_FOLDER=/home/$USER/Softwares
export EAGER_CONDA=$1

# Download EagerMOT Packages
cd ${SOFTWARE_PARENT_FOLDER}
git clone https://github.com/aleksandrkim61/EagerMOT
cd EagerMOT

# Create Conda Env
#echo "source ${SCRIPTS_FOLDER}/start_conda_local.sh" >> /home/${USER}/.bashrc
source ${SCRIPTS_FOLDER}/start_conda_local.sh
conda create --name ${EAGER_CONDA} python=3.7 -y
source ${SCRIPTS_FOLDER}/start_conda_local.sh
conda activate ${EAGER_CONDA}
python3 -m pip install -r requirements_pip.txt
python3 -m pip uninstall torch torchvision -y
source ${SCRIPTS_FOLDER}/start_conda_local.sh
conda activate ${EAGER_CONDA}
conda install pytorch=1.4.0 torchvision=0.5.0 cudatoolkit=10.0 -c pytorch -y

# Install Additional Packages
python3 -m pip install imageio ujson
python3 -m pip install -U llvmlite==0.32.1
python3 -m pip install --upgrade numba==0.56.0

# Create link of Required Files
mkdir data
cd data
ln -s ${EXT_DATA_PATH}/kitti/ .
cd ..
mkdir work_dir
cd work_dir
mkdir kitti_dir
cd kitti_dir
mkdir training
cd training
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_data/ego_motion/ .
cd ../..
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_data/ab3dmot/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_data/ego_motion_testing/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_data/pointgnn/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_data/mmdetection_cascade_x101/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_downloads/detections_segmentations_RRC_BB2SegNet/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_downloads/detections_segmentations_RRC_BB2SegNet_test/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_downloads/detections_segmentations_trackrcnn_BB2SegNet/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_downloads/detections_segmentations_trackrcnn_BB2SegNet_test/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_downloads/trackrcnn_detections/ .

# Copy Updated files
cd ..
cp ${SCRIPTS_FOLDER}/eagermot_scripts/adapt_kitti_motsfusion_input.py .
cp ${SCRIPTS_FOLDER}/eagermot_scripts/utils.py inputs/.
cp ${SCRIPTS_FOLDER}/eagermot_scripts/local_variables.py configs/.

# For the first run only
# python3 adapt_kitti_motsfusion_input.py

# To Run the Script
# python3 run_tracking.py

cd ${SOFTWARE_PARENT_FOLDER}