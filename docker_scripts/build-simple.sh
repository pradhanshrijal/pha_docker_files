#!/bin/bash

DOC_IMG="phaenvs/pha-22"
DOC_IMG_NAME="pha-22"
DOC_TAG=$(cat envs/$DOC_IMG_NAME/ENV_VERSION)

while getopts i:n:t: flag
do
    case "${flag}" in
        i) DOC_IMG=${OPTARG};;
        n) DOC_IMG_NAME=${OPTARG};;
        t) DOC_TAG=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker build -t ${DOC_IMG}:${DOC_TAG} -f envs/$DOC_IMG_NAME/Dockerfile --no-cache .