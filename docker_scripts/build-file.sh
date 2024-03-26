#!/bin/bash

DOC_IMG="phaenvs/pha-22"
DOC_TAG="sample"
DOCKERFILE_PATH="envs/pha-22/Dockerfile"

while getopts i:f:t: flag
do
    case "${flag}" in
        i) DOC_IMG=${OPTARG};;
        f) DOCKERFILE_PATH=${OPTARG};;
        t) DOC_TAG=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker build -t ${DOC_IMG}:${DOC_TAG} -f ${DOCKERFILE_PATH} --no-cache .