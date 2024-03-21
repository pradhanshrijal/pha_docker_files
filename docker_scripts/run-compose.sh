#!/bin/bash

ENV_FILE="docker_scripts/compose-file.env"
DOC_FILE="docker_scripts/docker-compose.yaml"
DOC_COMMAND="up -d"

while getopts e:f:o: flag
do
    case "${flag}" in
        e) ENV_FILE=${OPTARG};;
        f) DOC_FILE=${OPTARG};;
        o) DOC_COMMAND=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker compose --verbose --env-file ${ENV_FILE} -f ${DOC_FILE} ${DOC_COMMAND}