#!/bin/bash

ENV_FILE="${PHA_HOME}/envs/pha-22/pha-22.env"

while getopts e: flag
do
    case "${flag}" in
        e) ENV_FILE=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

set -a
source ${REQUIREMENTS_ENVS}

docker build -t ${DOC_IMG}:${DOC_TAG} -f ${DOCKERFILE_PATH} \
        --build-arg="IAMGE_NAME=${IMAGE_NAME}" \
        --build-arg="IMAGE_VERSION=${IMAGE_VERSION}" \
        --build-arg="NV_USERNAME=${NV_USERNAME}" \
        --build-arg="NV_CUDA_VERSION_NUMBER=${NV_CUDA_VERSION_NUMBER}" \
        --build-arg="NV_UBUNTU_VERSION=${NV_UBUNTU_VERSION}" \
        --build-arg="NV_CUDNN_VERSION=${NV_CUDNN_VERSION}" \
        --build-arg="NV_CUDNN_PACKAGE_NAME=${NV_CUDNN_PACKAGE_NAME}" \
        --build-arg="NV_TRT=${NV_TRT}" \
        --build-arg="PROJECT_NAME=${PROJECT_NAME}" \
        --build-arg="APT_GET_REQUIREMENTS_FILE=${APT_GET_REQUIREMENTS_FILE}" \
        --build-arg="PYTHON_REQUIREMENTS_FILE=${PYTHON_REQUIREMENTS_FILE}" \
        --build-arg="SCRIPT_REQUIREMENTS_FILE=${SCRIPT_REQUIREMENTS_FILE}" \
        --build-arg="NV_SSI_PATH=${NV_SSI_PATH}" \
        --build-arg="NV_SCRIPTS_PATH=${NV_SCRIPTS_PATH}" \
        --build-arg="NV_SOFTWARES_PATH=${NV_SOFTWARES_PATH}" \
        --build-arg="USER_REQUIREMENTS_PATH=${USER_REQUIREMENTS_PATH}" \
        --build-arg="NV_ROS_VERSION=${NV_ROS_VERSION}" \
        --no-cache .

set +a