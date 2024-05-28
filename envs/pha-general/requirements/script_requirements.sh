IN_USERNAME=$1
IN_SCRIPTS_PATH=$2
IN_ROS_VERSION=$3
IN_CUDA_VERSION_NUMBER=$4
IN_UBUNTU_VERSION=$5
IN_CUDNN_VERSION=$6
IN_CUDNN_PACKAGE_NAME=$7
IN_TRT=$8
IN_USERNAME="${IN_USERNAME:=pha}"
IN_SCRIPTS_PATH="${IN_SCRIPTS_PATH:=/home/${IN_USERNAME}/docker_share/scripts}"
IN_ROS_VERSION="${IN_ROS_VERSION:=humble}"
IN_CUDA_VERSION_NUMBER="${IN_CUDA_VERSION_NUMBER:=11.7}"
IN_UBUNTU_VERSION="${IN_UBUNTU_VERSION:=2204}"
IN_CUDNN_VERSION=${IN_CUDNN_VERSION:=8.5.0.96}
IN_CUDNN_PACKAGE_NAME=${IN_CUDNN_PACKAGE_NAME:=libcudnn8}
IN_TRT="${IN_TRT:=8.6.1}"

IN_CUDA_VERSION=cuda-${IN_CUDA_VERSION_NUMBER}

# Setup Display
echo -e "\n# Setup" >> /home/${IN_USERNAME}/.bashrc
echo "export USER=${IN_USERNAME}" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SCRIPTS_PATH}/setup/term_disp.sh" >> /home/${IN_USERNAME}/.bashrc

# Setup Source Paths
echo "source ${IN_SCRIPTS_PATH}/setup/export_docker_paths.sh" >> /home/${IN_USERNAME}/.bashrc

# Install CuDNN
source ${IN_SCRIPTS_PATH}/install/install_cudnn.sh ${IN_CUDA_VERSION_VERSION} ${IN_CUDNN_VERSION} ${IN_CUDNN_PACKAGE_NAME}

# CUDA Paths
echo "source ${IN_SCRIPTS_PATH}/setup/cuda_paths.sh ${IN_CUDA_VERSION}" >> /home/${IN_USERNAME}/.bashrc 

# Install TensorRT
source ${IN_SCRIPTS_PATH}/install/install_tensorrt.sh ${IN_UBUNTU_VERSION} ${IN_TRT}

# Install Conda
source ${IN_SCRIPTS_PATH}/install/install_conda.sh

# Install ROS
source ${IN_SCRIPTS_PATH}/install/install_ros.sh ${IN_USERNAME} ${IN_ROS_VERSION}

# Source Conda
echo "# source ${IN_SCRIPTS_PATH}/start_conda_local.sh" >> /home/${IN_USERNAME}/.bashrc