IN_USERNAME=$1
IN_ROS_VERSION=$2
IN_CARLA_VERSION=$3
IN_CUDA_FOLDER=$4
IN_SCRIPTS_PATH=$5
IN_USERNAME="${IN_USERNAME:=pha}"
IN_ROS_VERSION="${IN_ROS_VERSION:=humble}"
IN_CARLA_VERSION="${IN_CARLA_VERSION:=0.9.15}"
IN_CUDA_FOLDER="${IN_CUDA_FOLDER:=cuda-11.7}"
IN_SCRIPTS_PATH="${IN_SCRIPTS_PATH:=/home/${IN_USERNAME}/docker_share/scripts}"

# Setup Display
echo "# Setup" >> /home/${IN_USERNAME}/.bashrc
echo "export USER=${IN_USERNAME}" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SCRIPTS_PATH}/setup/term_disp.sh" >> /home/${IN_USERNAME}/.bashrc

# Setup Source Paths
echo "source ${IN_SCRIPTS_PATH}/setup/export_docker_paths.sh" >> /home/${IN_USERNAME}/.bashrc

# CUDA Paths
echo "source ${IN_SCRIPTS_PATH}/setup/cuda_paths.sh ${IN_CUDA_FOLDER}" >> /home/${IN_USERNAME}/.bashrc 

# Install Vulkan
source ${IN_SCRIPTS_PATH}/install/install_vulkan.sh

# Install ROS Humble Mini
source ${IN_SCRIPTS_PATH}/install/install_ros_mini.sh ${IN_USERNAME} ${IN_ROS_VERSION}

# Install Carla Dependencies
source ${IN_SCRIPTS_PATH}/install/install_carla_deps.sh ${IN_USERNAME} ${IN_ROS_VERSION} ${IN_CARLA_VERSION}