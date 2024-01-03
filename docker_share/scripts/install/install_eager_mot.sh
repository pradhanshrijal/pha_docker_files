#!/bin/bash

# File Tested with CUDA 10.0 in Ubuntu 18.04
# Input is the name of the Conda Container - ex: eager_conda

# To Change variables and environments, find all instance in the file and repalce them
export EXT_DATA_PATH=/media/$USER/storage_drive
export SCRIPTS_FOLDER=/media/$USER/storage_drive/software_files/eagermot_files/eagermot_scripts
export SOFTWARE_PARENT_FOLDER=/home/$USER/docker_share/git_pkgs

# Download EagerMOT Packages
cd ${SOFTWARE_PARENT_FOLDER}
git clone https://github.com/pradhanshrijal/EagerMOT
cd EagerMOT

# Create Conda Env
#echo "source ${SCRIPTS_FOLDER}/start_conda_local.sh" >> /home/${USER}/.bashrc
python3 -m pip install -r requirements_pip.txt

# Install Additional Packages
python3 -m pip install imageio ujson open3d

# Create link of Required Files
mkdir data
cd data
ln -s ${EXT_DATA_PATH}/datasets/Kitti_Datasets/kitti_eval_tracking/ .
cd ..
mkdir work_dir
cd work_dir
mkdir kitti_dir
cd kitti_dir
mkdir training
cd training
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_data/training/ego_motion/ .
cd ..
mkdir testing
cd testing
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_data/testing/ego_motion/ .
cd ..
mkdir detections
cd detections
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_data/ab3dmot/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_data/pointgnn/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_data/mmdetection_cascade_x101/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_detections/detections_segmentations_RRC_BB2SegNet/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_detections/detections_segmentations_RRC_BB2SegNet_test/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_detections/detections_segmentations_trackrcnn_BB2SegNet/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_detections/detections_segmentations_trackrcnn_BB2SegNet_test/ .
ln -s ${EXT_DATA_PATH}/software_files/eagermot_files/eagermot_detections/trackrcnn_detections/ .

# Copy Updated files
cd ../../..
cp ${SCRIPTS_FOLDER}/adapt_kitti_motsfusion_input.py .
cp ${SCRIPTS_FOLDER}/utils.py inputs/.
cp ${SCRIPTS_FOLDER}/local_variables.py configs/.

# For the first run only
# python3 adapt_kitti_motsfusion_input.py

# To Run the Script
# python3 run_tracking.py

cd ${SOFTWARE_PARENT_FOLDER}