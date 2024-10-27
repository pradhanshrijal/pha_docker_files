#!/bin/bash

DOC_IMG="phaenvs/pha-22"
DOC_TAG="latest"
DOCKERFILE_PATH="envs/custom-user/Dockerfile"
USERNAME=${USER}
CUSTOM_NAME="phaenvs/pha-22"
CUSTOM_TAG="custom-${USERNAME}"

UID_VAR=$(id -u)
GID_VAR=$(id -g)

while getopts d:f:g:i:m:t:u:z: flag
do
    case "${flag}" in
        d) UID_VAR=${OPTARG};;
        f) DOCKERFILE_PATH=${OPTARG};;
        g) GID_VAR=${OPTARG};;
        i) DOC_IMG=${OPTARG};;
        m) CUSTOM_NAME=${OPTARG};;
        t) DOC_TAG=${OPTARG};;
        u) USERNAME=${OPTARG};;
        z) CUSTOM_TAG=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker build --build-arg IMAGE_NAME=${DOC_IMG} --build-arg IMAGE_VERSION=${DOC_TAG} --build-arg UID=${UID_VAR} --build-arg GID=${GID_VAR} --build-arg USERNAME=$USERNAME -t ${CUSTOM_NAME}:${CUSTOM_TAG} -f ${DOCKERFILE_PATH} --no-cache .