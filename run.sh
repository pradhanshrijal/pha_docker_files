#!/bin/bash
# Usage: source run.sh ${DOC_IMGAINER_NAME}
DOC_IMG_NAME=$1
CONT_NAME=$2
DOC_IMG_NAME="${DOC_IMG_NAME:=pha-22}"
CONT_NAME="${CONT_NAME:=${DOC_IMG_NAME}}"
ENV_VERSION=$(cat envs/$DOC_IMG_NAME/ENV_VERSION)

docker run -d --name ${CONT_NAME} -e DISPLAY=$DISPLAY --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all  --env=QT_X11_NO_MITSHM=1 --runtime=nvidia --privileged --shm-size=16gb -v /tmp/.X11-unix:/tmp/.X11-unix --network host -v /home/${USER}/schreibtisch/pha_docker_files/docker_share:/home/pha/docker_share -v /media/${USER}:/media/pha -v /dev:/dev --gpus all -it phaenvs/${DOC_IMG_NAME}:${ENV_VERSION} /bin/bash