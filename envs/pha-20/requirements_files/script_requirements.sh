IN_USERNAME=$1
IN_ROS_VERSION=$2
IN_CUDA_FOLDER=$3
IN_UBUNTU_VERSION=$4
IN_TRT=$5
IN_SCRIPTS_PATH=$6
IN_USERNAME="${IN_USERNAME:=pha}"
IN_ROS_VERSION="${IN_ROS_VERSION:=foxy}"
IN_CUDA_FOLDER="${IN_CUDA_FOLDER:=cuda-11.3}"
IN_UBUNTU_VERSION="${IN_UBUNTU_VERSION:=2004}"
IN_TRT="${IN_TRT:=8.6.1}"
IN_SCRIPTS_PATH="${IN_SCRIPTS_PATH:=/home/${IN_USERNAME}/docker_share/scripts}"

# Setup Display
echo -e "\n# Setup" >> /home/${IN_USERNAME}/.bashrc
echo "export USER=${IN_USERNAME}" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SCRIPTS_PATH}/setup/term_disp.sh" >> /home/${IN_USERNAME}/.bashrc

# Setup Source Paths
echo "source ${IN_SCRIPTS_PATH}/setup/export_docker_paths.sh" >> /home/${IN_USERNAME}/.bashrc

# CUDA Paths
echo "source ${IN_SCRIPTS_PATH}/setup/cuda_paths.sh ${IN_CUDA_FOLDER}" >> /home/${IN_USERNAME}/.bashrc 

# Install TensorRT
source ${IN_SCRIPTS_PATH}/install/install_tensorrt.sh ${IN_UBUNTU_VERSION} ${IN_TRT}

# Install Conda
source ${IN_SCRIPTS_PATH}/install/install_conda.sh

# Install ROS
source ${IN_SCRIPTS_PATH}/install/install_ros.sh ${IN_USERNAME} ${IN_ROS_VERSION}

# Source Conda
echo "# source ${IN_SCRIPTS_PATH}/start_conda_local.sh" >> /home/${IN_USERNAME}/.bashrc