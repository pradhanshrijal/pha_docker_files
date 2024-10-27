#!/bin/bash

ENV_FILE="${PHA_HOME}/envs/pha-22/pha-22.env"

UID_VAR=$(id -u)
GID_VAR=$(id -g)

while getopts d:e:g: flag
do
    case "${flag}" in
        d) UID_VAR=${OPTARG};;
        e) ENV_FILE=${OPTARG};;
        g) GID_VAR=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

set -a
export UID_VAR=${UID_VAR}
export GID_VAR=${GID_VAR}
source ${ENV_FILE}

docker run -d --name ${CONT_NAME} \
                -e DISPLAY=${DISPLAY_VAR} \
                --env=NVIDIA_VISIBLE_DEVICES=all \
                --env=NVIDIA_DRIVER_CAPABILITIES=all \
                --env=QT_X11_NO_MITSHM=1 \
                --runtime=nvidia \
                --privileged \
                --shm-size=${SHM_SIZE} \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                --network host \
                -v ${SSI_PATH}:/home/${IN_USERNAME}/docker_share \
                -v /media/${USER}:/media/${IN_USERNAME} \
                -v /dev:/dev \
                --gpus all \
                -it ${DOC_IMG}:${DOC_TAG} /bin/bash