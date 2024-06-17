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

docker build -t ${DOC_IMG}:${DOC_TAG} -f ${DOCKERFILE_PATH} \
        --build-arg="IMAGE_NAME=${IMAGE_NAME}" \
        --build-arg="IMAGE_VERSION=${IMAGE_VERSION}" \
        --build-arg="IN_USERNAME=${IN_USERNAME}" \
        --build-arg="UID=${UID_VAR}" \
        --build-arg="GID=${GID_VAR}" \
        --build-arg="IN_CUDA_VERSION_NUMBER=${IN_CUDA_VERSION_NUMBER}" \
        --build-arg="IN_UBUNTU_VERSION=${IN_UBUNTU_VERSION}" \
        --build-arg="IN_CUDNN_VERSION=${IN_CUDNN_VERSION}" \
        --build-arg="IN_CUDNN_PACKAGE_NAME=${IN_CUDNN_PACKAGE_NAME}" \
        --build-arg="IN_TRT=${IN_TRT}" \
        --build-arg="PROJECT_NAME=${PROJECT_NAME}" \
        --build-arg="APT_GET_REQUIREMENTS_FILE=${APT_GET_REQUIREMENTS_FILE}" \
        --build-arg="PYTHON_REQUIREMENTS_FILE=${PYTHON_REQUIREMENTS_FILE}" \
        --build-arg="SCRIPT_REQUIREMENTS_FILE=${SCRIPT_REQUIREMENTS_FILE}" \
        --build-arg="IN_SSI_PATH=${IN_SSI_PATH}" \
        --build-arg="IN_SCRIPTS_PATH=${IN_SCRIPTS_PATH}" \
        --build-arg="IN_SOFTWARES_PATH=${IN_SOFTWARES_PATH}" \
        --build-arg="USER_REQUIREMENTS_PATH=${USER_REQUIREMENTS_PATH}" \
        --build-arg="IN_ROS_VERSION=${IN_ROS_VERSION}" \
        --no-cache .

set +a