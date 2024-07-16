IN_USERNAME=$1
IN_ROS_VERSION=$2
IN_CARLA_VERSION=$3
IN_CUDA_VERSION_NUMBER=$4
IN_SSI_PATH=$5
IN_USERNAME="${IN_USERNAME:=pha}"
IN_ROS_VERSION="${IN_ROS_VERSION:=humble}"
IN_CARLA_VERSION="${IN_CARLA_VERSION:=0.9.15}"
IN_CUDA_VERSION_NUMBER="${IN_CUDA_VERSION_NUMBER:=11.7}"
IN_SSI_PATH="${IN_SSI_PATH:=/home/${IN_USERNAME}/docker_share}"

IN_CUDA_VERSION=cuda-${IN_CUDA_VERSION_NUMBER}

# Setup Display
echo "# Setup" >> /home/${IN_USERNAME}/.bashrc
echo "export USER=${IN_USERNAME}" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SSI_PATH}/scripts/setup/term_disp.sh" >> /home/${IN_USERNAME}/.bashrc

# Setup Source Paths
echo "source ${IN_SSI_PATH}/scripts/setup/export_docker_paths.sh" >> /home/${IN_USERNAME}/.bashrc

# CUDA Paths
echo "source ${IN_SSI_PATH}/scripts/setup/cuda_paths.sh ${IN_CUDA_VERSION}" >> /home/${IN_USERNAME}/.bashrc 

# Install Vulkan
source ${IN_SSI_PATH}/scripts/install/install_vulkan.sh

# Install ROS Humble Mini
source ${IN_SSI_PATH}/scripts/install/install_ros_mini.sh ${IN_USERNAME} ${IN_ROS_VERSION}

# Install Carla Dependencies
source ${IN_SSI_PATH}/scripts/install/install_carla_deps.sh ${IN_USERNAME} ${IN_ROS_VERSION} ${IN_CARLA_VERSION}