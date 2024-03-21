#!/bin/bash

DOC_IMG="phaenvs/pha-22"
DOC_TAG="latest"
USERNAME=${USER}
CUSTOM_NAME="phaenvs/pha-22"
CUSTOM_TAG="custom-${USERNAME}"

UID=$(id -u)
GID=$(id -g)

while getopts d:g:i:m:s:t:u: flag
do
    case "${flag}" in
        d) UID=${UID};;
        g) GID=${UID};;
        i) DOC_IMG=${OPTARG};;
        m) CUSTOM_NAME=${OPTARG};;
        s) CUSTOM_TAG=${OPTARG};;
        t) DOC_TAG=${OPTARG};;
        u) USERNAME=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker build --build-arg IMAGE_NAME=${DOC_IMG} --build-arg IMAGE_VERSION=${DOC_TAG} --build-arg UID=${UID} --build-arg GID=${GID} --build-arg USERNAME=$USERNAME -t ${CUSTOM_NAME}:${CUSTOM_TAG} -f envs/custom-user/Dockerfile --no-cache .