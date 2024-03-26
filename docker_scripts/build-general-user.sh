#!/bin/bash

DOC_IMG="ubuntu"
DOC_TAG="22.04"
USERNAME=${USER}
CUSTOM_NAME="ubuntu-22"
CUSTOM_TAG="custom-${USERNAME}"

UID=$(id -u)
GID=$(id -g)

while getopts a:d:g:i:m:t:u: flag
do
    case "${flag}" in
        a) CUSTOM_TAG=${OPTARG};;
        d) UID=${UID};;
        g) GID=${UID};;
        i) DOC_IMG=${OPTARG};;
        m) CUSTOM_NAME=${OPTARG};;
        t) DOC_TAG=${OPTARG};;
        u) USERNAME=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker build --build-arg IMAGE_NAME=${DOC_IMG} --build-arg IMAGE_VERSION=${DOC_TAG} --build-arg UID=${UID} --build-arg GID=${GID} --build-arg USERNAME=$USERNAME -t ${CUSTOM_NAME}:${CUSTOM_TAG} -f envs/custom-general-user/Dockerfile --no-cache .