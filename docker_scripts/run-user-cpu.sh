#!/bin/bash

DOC_IMG="phaenvs/pha-22"
CONT_NAME="pha-22"
DOC_TAG="latest"
SSI_PATH="/home/${USER}/schreibtisch/pha_docker_files/docker_share"
USERNAME=${USER}

while getopts c:i:s:t:u: flag
do
    case "${flag}" in
        c) CONT_NAME=${OPTARG};;
        i) DOC_IMG=${OPTARG};;
        s) SSI_PATH=${OPTARG};;
        t) DOC_TAG=${OPTARG};;
        u) USERNAME=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker run -d --name ${CONT_NAME} \
                -e DISPLAY=$DISPLAY \
                --env=QT_X11_NO_MITSHM=1 \
                --privileged \
                --shm-size=16gb \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                --network host \
                -v ${SSI_PATH}:/home/${USERNAME}/docker_share \
                -v /media/${USERNAME}:/media/${USERNAME} \
                -v /dev:/dev \
                -it ${DOC_IMG}:${DOC_TAG} /bin/bash