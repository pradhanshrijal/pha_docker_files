# Commands
# 1. Create Docker Image (Execute in the parent directory)
# docker build -t phaenvs/jekyll-22:0.1.0 -f envs/jekyll-22/Dockerfile --no-cache .
# 2. Run Docker Image (Execute in the parent directory)
# xhost +local:docker
# docker run -d --name jekyll-22 -e DISPLAY=$DISPLAY --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all  --env=QT_X11_NO_MITSHM=1 --runtime=nvidia --privileged --shm-size=16gb -v /tmp/.X11-unix:/tmp/.X11-unix --network host -v /home/${USER}/schreibtisch/pha_docker_files/docker_share:/home/pha/docker_share -v /media/${USER}:/media/pha -v /dev:/dev --gpus all -it phaenvs/jekyll-22:0.1.0 /bin/bash
# or
# docker compose -f envs/jekyll-22/docker-compose.yaml up -d

# Declare VARIABLES
ARG IMAGE_NAME=ubuntu
ARG IMAGE_VERSION=22.04
ARG DEBIAN_FRONTEND=noninteractive

FROM ${IMAGE_NAME}:${IMAGE_VERSION} as base
FROM base as base-amd64

ENV NV_USERNAME=pha
ENV NV_UBUNTU_VERSION=2204

ARG PROJECT_NAME=jekyll-22

ARG APT_GET_REQUIREMENTS_FILE=apt-get_requirements.txt
ARG SCRIPT_REQUIREMENTS_FILE=script_requirements.sh

ARG NV_SSI_PATH=/home/${NV_USERNAME}/docker_share
ARG NV_SCRIPTS_PATH=${NV_SSI_PATH}/scripts
ARG NV_SOFTWARES_PATH=/home/${NV_USERNAME}/Softwares
ARG USER_REQUIREMENTS_PATH=envs/${PROJECT_NAME}/requirements_files
# ---------------------------------------------------------

# Declare Time Zone
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
#

# Update and install basics
RUN apt update && apt upgrade -y
RUN apt install sudo -y
#

# Setup user
SHELL ["/bin/bash", "-c"]
RUN ["/bin/bash", "-c", "sudo useradd -m ${NV_USERNAME}"]
RUN ["/bin/bash", "-c", "sudo usermod -aG sudo ${NV_USERNAME}"]

WORKDIR /home/
RUN ["/bin/bash", "-c", "sudo chown -R ${NV_USERNAME}:${NV_USERNAME} /home/${NV_USERNAME}"]
WORKDIR /home/${NV_USERNAME}
RUN ["/bin/bash", "-c", "sudo chsh -s /bin/bash ${NV_USERNAME}"]
RUN mkdir ${NV_SSI_PATH}
RUN mkdir ${NV_SCRIPTS_PATH}

## Copy Scripts
RUN mkdir ${NV_SCRIPTS_PATH}/setup
COPY docker_share/scripts/setup ${NV_SCRIPTS_PATH}/setup
RUN mkdir ${NV_SCRIPTS_PATH}/install
COPY docker_share/scripts/install ${NV_SCRIPTS_PATH}/install
RUN mkdir ${NV_SCRIPTS_PATH}/tmp
COPY ${USER_REQUIREMENTS_PATH}/${APT_GET_REQUIREMENTS_FILE} ${NV_SCRIPTS_PATH}/tmp/
COPY ${USER_REQUIREMENTS_PATH}/${SCRIPT_REQUIREMENTS_FILE} ${NV_SCRIPTS_PATH}/tmp/
#RUN mkdir ${NV_SCRIPTS_PATH}/functions
#COPY docker_share/scripts/functions ${NV_SCRIPTS_PATH}/functions
#RUN mkdir ${NV_SCRIPTS_PATH}/vars
#COPY docker_share/scripts/vars ${NV_SCRIPTS_PATH}/vars
RUN mkdir ${NV_SOFTWARES_PATH}

RUN sudo cp /root/.bashrc /home/${NV_USERNAME}/.
RUN echo "source ${NV_SCRIPTS_PATH}/setup/term_disp.sh" >> /root/.bashrc
RUN sudo chown -R ${NV_USERNAME}:${NV_USERNAME} /home/${NV_USERNAME}/.bashrc
RUN sudo chown -R ${NV_USERNAME}:${NV_USERNAME} ${NV_SSI_PATH}
RUN echo "${NV_USERNAME} ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers
##

## Switch to User
RUN ["/bin/bash", "-c", "source /root/.bashrc"]
USER ${NV_USERNAME}
##

# End - Setup user

# Setup Programmes

## Setup apt-get
RUN sudo apt-get update
RUN sudo apt-get install --no-install-recommends -y checkinstall
RUN xargs sudo apt-get install --no-install-recommends -y < ${NV_SCRIPTS_PATH}/tmp/${APT_GET_REQUIREMENTS_FILE}
##

# End - Setup Programmes

# Install based on scripts
WORKDIR "${NV_SOFTWARES_PATH}"
RUN ["/bin/bash", "-c", "source /home/${NV_USERNAME}/.bashrc"]
RUN ["/bin/bash", "-c", "source ${NV_SCRIPTS_PATH}/tmp/${SCRIPT_REQUIREMENTS_FILE}"]
# End - Install based on scripts

# Remove Cache
RUN ["/bin/bash", "-c", "rm -rf /home/${NV_USERNAME}/.cache/*"]
RUN ["/bin/bash", "-c", "rm -rf ${NV_SCRIPTS_PATH}/tmp"]
#

WORKDIR "/home/${NV_USERNAME}"
