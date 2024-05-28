# Commands
# 1. Create Docker Image (Execute in the parent directory)
# docker build -t phaenvs/pha-22:custom-user -f envs/custom-user/Dockerfile --no-cache .
# 2. Run Docker Image (Execute in the parent directory)
# xhost +local:docker
# docker run -d --name pha-22 -e DISPLAY=$DISPLAY --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all  --env=QT_X11_NO_MITSHM=1 --runtime=nvidia --privileged --shm-size=16gb -v /tmp/.X11-unix:/tmp/.X11-unix --network host -v /home/${USER}/schreibtisch/pha_docker_files/docker_share:/home/pha/docker_share -v /media/${USER}:/media/pha -v /dev:/dev --gpus all -it phaenvs/pha-22:0.1.0 /bin/bash
# or
# docker compose -f envs/pha-22/docker-compose.yaml up -d

# Declare VARIABLES
ARG IMAGE_NAME=phaenvs/pha-22
ARG IMAGE_VERSION=latest
ARG DEBIAN_FRONTEND=noninteractive

FROM ${IMAGE_NAME}:${IMAGE_VERSION} as base
FROM base as base-amd64

ARG USERNAME=devuser
ARG IN_CUDA_VERSION_NUMBER=11.7
ARG UID=${UID}
ARG GID=${GID}

ARG PROJECT_NAME=custom-user

ARG APT_GET_REQUIREMENTS_FILE=apt-get_requirements.txt
ARG PYTHON_REQUIREMENTS_FILE=python_requirements.txt
ARG SCRIPT_REQUIREMENTS_FILE=script_requirements.sh

ARG IN_SSI_PATH=/home/${USERNAME}/docker_share
ARG IN_SCRIPTS_PATH=${IN_SSI_PATH}/scripts
ARG USER_REQUIREMENTS_PATH=envs/${PROJECT_NAME}/requirements_files

ARG IN_ROS_VERSION=humble
#

# Remove User
USER root
RUN userdel pha
#

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
    && useradd --uid ${UID} --gid ${GID} --create-home ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME} \
    && chown -R ${UID}:${GID} /home/${USERNAME}

WORKDIR "/home/${USERNAME}"

## Copy Scripts
RUN mkdir ${IN_SCRIPTS_PATH}/setup
COPY docker_share/scripts/setup ${IN_SCRIPTS_PATH}/setup
RUN mkdir ${IN_SCRIPTS_PATH}/install
COPY docker_share/scripts/install ${IN_SCRIPTS_PATH}/install
RUN mkdir ${IN_SCRIPTS_PATH}/tmp
COPY ${USER_REQUIREMENTS_PATH}/${APT_GET_REQUIREMENTS_FILE} ${IN_SCRIPTS_PATH}/tmp/
COPY ${USER_REQUIREMENTS_PATH}/${PYTHON_REQUIREMENTS_FILE} ${IN_SCRIPTS_PATH}/tmp/
COPY ${USER_REQUIREMENTS_PATH}/${SCRIPT_REQUIREMENTS_FILE} ${IN_SCRIPTS_PATH}/tmp/
##

# End - New User

# Setup Programmes

## Setup apt-get
RUN sudo apt-get update
RUN sudo apt-get install --no-install-recommends -y checkinstall
RUN xargs sudo apt-get install --no-install-recommends -y < ${IN_SCRIPTS_PATH}/tmp/${APT_GET_REQUIREMENTS_FILE}
##

## Setup Python
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install -r ${IN_SCRIPTS_PATH}/tmp/${PYTHON_REQUIREMENTS_FILE}
##

# End - Setup Programmes

# Install based on scripts
RUN ["/bin/bash", "-c", "source /home/${USERNAME}/.bashrc"]
RUN ["/bin/bash", "-c", "source ${IN_SCRIPTS_PATH}/tmp/${SCRIPT_REQUIREMENTS_FILE} ${USERNAME} ${IN_SCRIPTS_PATH} ${IN_ROS_VERSION} ${IN_CUDA_VERSION_NUMBER}"]
# End - Install based on scripts

# Remove Cache
RUN ["/bin/bash", "-c", "rm -rf /home/${USERNAME}/.cache/*"]
RUN ["/bin/bash", "-c", "rm -rf ${IN_SCRIPTS_PATH}/tmp"]
#