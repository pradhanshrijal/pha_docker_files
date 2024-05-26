#!/bin/bash

DOC_IMG="phaenvs/pha-22"
DOC_TAG="sample"
DOCKERFILE_PATH="envs/pha-22/Dockerfile"
PYTHONFILE_NAME="python_requirements.txt"
APTGETFILE_NAME="apt-get_requirements.txt"
SCRIPTFILE_NAME="script_requirements.sh"

while getopts i:f:p:q:r:t: flag
do
    case "${flag}" in
        i) DOC_IMG=${OPTARG};;
        f) DOCKERFILE_PATH=${OPTARG};;
        p) PYTHONFILE_NAME=${OPTARG};;
        q) APTGETFILE_NAME=${OPTARG};;
        r) SCRIPTFILE_NAME=${OPTARG};;
        t) DOC_TAG=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker build -t ${DOC_IMG}:${DOC_TAG} -f ${DOCKERFILE_PATH} \
        --build-arg="PYTHON_REQUIREMENTS_FILE=${PYTHONFILE_NAME}" \
        --build-arg="APT_GET_REQUIREMENTS_FILE=${APTGETFILE_NAME}" \
        --build-arg="SCRIPT_REQUIREMENTS_FILE=${SCRIPTFILE_NAME}" \
        --no-cache .