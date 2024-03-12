# Commands
# 1. Create Docker Image (Execute in the parent directory)
# docker build -t phaenvs/pha-22:0.1.0 -f envs/pha-22/Dockerfile --no-cache .
# 2. Run Docker Image (Execute in the parent directory)
# xhost +local:docker
# docker run -d --name pha-22 -e DISPLAY=$DISPLAY --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all  --env=QT_X11_NO_MITSHM=1 --runtime=nvidia --privileged --shm-size=16gb -v /tmp/.X11-unix:/tmp/.X11-unix --network host -v /home/${USER}/schreibtisch/pha_docker_files/docker_share:/home/pha/docker_share -v /media/${USER}:/media/pha -v /dev:/dev --gpus all -it phaenvs/pha-22:0.1.0 /bin/bash
# or
# docker compose -f envs/pha-22/docker-compose.yaml up -d

# Declare VARIABLES
ARG IMAGE_NAME=nvidia/cuda
ARG IMAGE_VERSION=11.7.1-devel-ubuntu22.04
ARG DEBIAN_FRONTEND=noninteractive

FROM ${IMAGE_NAME}:${IMAGE_VERSION} as base
FROM base as base-amd64

ENV NV_USERNAME=pha
ENV NV_CUDA_VERSION_NUMBER=11.7
ENV NV_UBUNTU_VERSION=2204
ENV NV_CUDNN_VERSION 8.5.0.96
ENV NV_CUDNN_PACKAGE_NAME libcudnn8
ENV NV_TRT=8.6.1

ENV NV_SCRIPTS_PATH=/home/${NV_USERNAME}/docker_share/scripts
ENV NV_SOFTWARES_PATH=/home/${NV_USERNAME}/Softwares

ENV NV_ROS_VERSION=humble
## If you change any variables, don't forge to check the script as well
## script is => docker_share/scripts/setup/variables_install_env.sh
# ---------------------------------------------------------
# Declare Time Zone
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Declare CUDA Environments
ENV NV_CUDA_VERSION=cuda${NV_CUDA_VERSION_NUMBER}
ENV NV_CUDA_FOLDER=cuda-${NV_CUDA_VERSION_NUMBER}

ENV NV_CUDNN_PACKAGE_VERSION ${NV_CUDNN_VERSION}-1

ENV NV_CUDNN_PACKAGE ${NV_CUDNN_PACKAGE_NAME}=${NV_CUDNN_PACKAGE_VERSION}+${NV_CUDA_VERSION}
ENV NV_CUDNN_PACKAGE_DEV ${NV_CUDNN_PACKAGE_NAME}-dev=${NV_CUDNN_PACKAGE_VERSION}+${NV_CUDA_VERSION}

# Update and install basics
RUN apt update && apt upgrade -y
RUN apt install sudo -y
RUN sudo apt install --no-install-recommends wget -y

RUN sudo apt install -y --no-install-recommends \
        ${NV_CUDNN_PACKAGE} \
        ${NV_CUDNN_PACKAGE_DEV}

RUN sudo apt-mark hold ${NV_CUDNN_PACKAGE_NAME}
RUN rm -rf /var/lib/apt/lists/*

# Install vim editor
RUN sudo apt update
RUN sudo apt install vim -y

# Setup user
SHELL ["/bin/bash", "-c"]
RUN ["/bin/bash", "-c", "sudo useradd -m ${NV_USERNAME}"]
RUN ["/bin/bash", "-c", "sudo usermod -aG sudo ${NV_USERNAME}"]

WORKDIR /home/
RUN ["/bin/bash", "-c", "sudo chown -R ${NV_USERNAME}:${NV_USERNAME} /home/${NV_USERNAME}"]
WORKDIR /home/${NV_USERNAME}
RUN ["/bin/bash", "-c", "sudo chsh -s /bin/bash ${NV_USERNAME}"]
RUN mkdir /home/${NV_USERNAME}/docker_share
RUN mkdir ${NV_SCRIPTS_PATH}

## Copy Scripts
RUN mkdir ${NV_SCRIPTS_PATH}/setup
COPY docker_share/scripts/setup ${NV_SCRIPTS_PATH}/setup
RUN mkdir ${NV_SCRIPTS_PATH}/install
COPY docker_share/scripts/install ${NV_SCRIPTS_PATH}/install
#RUN mkdir ${NV_SCRIPTS_PATH}/functions
#COPY docker_share/scripts/functions ${NV_SCRIPTS_PATH}/functions
#RUN mkdir ${NV_SCRIPTS_PATH}/vars
#COPY docker_share/scripts/vars ${NV_SCRIPTS_PATH}/vars

#RUN mkdir ${NV_SCRIPTS_PATH}/eagermot
#COPY docker_share/scripts/eagermot/ ${NV_SCRIPTS_PATH}/eagermot/

RUN sudo cp /root/.bashrc /home/${NV_USERNAME}/.
RUN echo -e "\n# Setup" \
            >> /home/${NV_USERNAME}/.bashrc
RUN echo "export USER=${NV_USERNAME}" \
            >> /home/${NV_USERNAME}/.bashrc
RUN echo "source ${NV_SCRIPTS_PATH}/setup/term_disp.sh" >> /root/.bashrc
RUN echo "source ${NV_SCRIPTS_PATH}/setup/term_disp.sh" \
            >> /home/${NV_USERNAME}/.bashrc
RUN sudo chown -R ${NV_USERNAME}:${NV_USERNAME} /home/${NV_USERNAME}/.bashrc
RUN echo "${NV_USERNAME} ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers
##

## Switch to User
RUN ["/bin/bash", "-c", "source /root/.bashrc"]
USER ${NV_USERNAME}
##

RUN mkdir ${NV_SOFTWARES_PATH}

# Setup Python
RUN sudo apt update
RUN sudo apt install net-tools python3 python3-pip \
                        vim python3-pyqt5 git git-lfs iputils-ping -y
#RUN python2 -m pip install --upgrade pip
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install virtualenv

# Install based on scripts
WORKDIR "${NV_SOFTWARES_PATH}"
## CUDA Paths
RUN echo "source ${NV_SCRIPTS_PATH}/setup/cuda_paths.sh ${NV_CUDA_FOLDER}" >> /home/${NV_USERNAME}/.bashrc 
##

## Install Sublime Text:
RUN ["/bin/bash", "-c", "source ${NV_SCRIPTS_PATH}/install/install_sublimetext.sh"]

# Install apt packages
RUN sudo apt install tmux nautilus -y

## Upgrade Cmake
RUN sudo apt install cmake g++ make -y
##

# Install TensorRT
RUN ["/bin/bash", "-c", "source ${NV_SCRIPTS_PATH}/install/install_tensorrt.sh ${NV_UBUNTU_VERSION} ${NV_TRT}"]

## Install Conda
RUN ["/bin/bash", "-c", "source ${NV_SCRIPTS_PATH}/install/install_conda.sh"]
##

## Install ROS Humble
RUN ["/bin/bash", "-c", "source /home/${NV_USERNAME}/.bashrc"]
RUN ["/bin/bash", "-c", "source ${NV_SCRIPTS_PATH}/install/install_ros.sh ${NV_USERNAME} ${NV_ROS_VERSION}"]
# RUN ["/bin/bash", "-c", "source ${NV_SCRIPTS_PATH}/install/install_autoware.sh /home/${NV_USERNAME}/ros2_ws"]
##

WORKDIR "/home/${NV_USERNAME}"

## Source Conda
# RUN echo "source ${NV_SCRIPTS_PATH}/start_conda_local.sh" >> /home/${NV_USERNAME}/.bashrc
##
