#!/bin/bash

ENV_FILE="docker_scripts/compose-file.env"
DOC_FILE="docker_scripts/docker-compose.yaml"
DOC_COMMAND="up -d"

UID_VAR=$(id -u)
GID_VAR=$(id -g)

export UID_VAR=${UID_VAR}
export GID_VAR=${GID_VAR}

while getopts d:e:f:g:o: flag
do
    case "${flag}" in
        d) UID_VAR=${OPTARG};;
        e) ENV_FILE=${OPTARG};;
        f) DOC_FILE=${OPTARG};;
        g) GID_VAR=${OPTARG};;
        o) DOC_COMMAND=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker compose --verbose --env-file ${ENV_FILE} -f ${DOC_FILE} ${DOC_COMMAND}