#!/bin/bash

ENV_FILE="docker_scripts/compose-file.env"
COM_FILE="docker_scripts/docker-compose.yaml"
COM_COMMAND="up -d"

UID_VAR=$(id -u)
GID_VAR=$(id -g)

while getopts b:d:e:g:o: flag
do
    case "${flag}" in
        b) COM_FILE=${OPTARG};;
        d) UID_VAR=${OPTARG};;
        e) ENV_FILE=${OPTARG};;
        g) GID_VAR=${OPTARG};;
        o) COM_COMMAND=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

export UID_VAR=${UID_VAR}
export GID_VAR=${GID_VAR}

docker compose --verbose --env-file ${ENV_FILE} -f ${COM_FILE} ${COM_COMMAND}