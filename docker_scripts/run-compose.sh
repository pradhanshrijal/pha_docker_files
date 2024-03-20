#!/bin/bash

ENV_FILE="docker_scripts/compose-file.env"
DOC_FILE="docker_scripts/docker-compose.yaml"
DOC_COMMAND="up -d"

while getopts c:e:f: flag
do
    case "${flag}" in
        c) DOC_COMMAND=${OPTARG};;
        e) ENV_FILE=${OPTARG};;
        f) DOC_FILE=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker compose --verbose --env-file ${ENV_FILE} -f ${DOC_FILE} ${DOC_COMMAND}