#!/bin/bash
# Usage: source run.sh ${DOC_IMAGE_NAME}
DOC_IMG="phaenvs/pha-22"
CONT_NAME="pha-22"
DOC_TAG="latest"
SSI_PATH="/home/${USER}/schreibtisch/pha_docker_files/docker_share"

while getopts i:c:s:t: flag
do
    case "${flag}" in
        i) DOC_IMG=${OPTARG};;
        c) CONT_NAME=${OPTARG};;
        s) SSI_PATH=${OPTARG};;
        t) DOC_TAG=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker run -d --name ${CONT_NAME} \
                -e DISPLAY=$DISPLAY \
                --env=NVIDIA_VISIBLE_DEVICES=all \
                --env=NVIDIA_DRIVER_CAPABILITIES=all \
                --env=QT_X11_NO_MITSHM=1 \
                --runtime=nvidia \
                --privileged \
                --shm-size=16gb \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                --network host \
                -v ${SSI_PATH}:/home/pha/docker_share \
                -v /media/${USER}:/media/pha \
                -v /dev:/dev \
                --gpus all \
                -it ${DOC_IMG}:${DOC_TAG} /bin/bash