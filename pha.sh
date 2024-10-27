#!/bin/bash

source ${PHA_HOME}/docker_scripts/pha_command_help.sh

DOC_IMG="phaenvs/pha-22"
DOC_TAG="sample"
DOCKERFILE_PATH="envs/pha-22/Dockerfile"
PYTHONFILE_NAME="python_requirements.txt"
APTGETFILE_NAME="apt-get_requirements.txt"
SCRIPTFILE_NAME="script_requirements.sh"

ENV_FILE="${PHA_HOME}/envs/pha-22/pha-22.env"
COM_FILE="docker_scripts/docker-compose.yaml"
COM_COMMAND="up -d"

UID_VAR=$(id -u)
GID_VAR=$(id -g)

ADDON_COMMAND=""
PHA_COMMAND=""
CURRENT_DIR="${PWD}"

show_help(){
    echo "Commands for PHA:

    active:             list all active contianers
    build:              build based on environmental files
    build-file:         build a Dockerfile with scripts
    compose:            user compose-file for pha setup
    containers:         list all containers
    enter:              start and execute an available container
    exec:               execute an active docker container
    flags:              show list of flags for PHA
    images:             list all available docker images
    purge:              remove all docker containers
    purge-imgs:         remvove all docker images
    purge-nl:           remove all nameless docker containers
    purge-nl-imgs:      remove all nameless docker images
    run:                run a container from environment files
    run-cpu:
    start:              start an available container

Helper Flags for PHA:
    -h | --help         show this information page
    
For command instructions: pha {COMMAND} -h"
}

show_command_help()
{
    case "${PHA_COMMAND}" in
    active)
        show_active_help
        ;;
    build)
        show_build_help
        ;;
    build-file)
        show_build_file_help
        ;;
    compose)
        show_compose_help
        ;;
    containers)
        show_containers_help
        ;;
    *)
        echo "Unknown option: $1"
        echo ""
        show_help
        exit 1
        ;;
    esac
    PHA_COMMAND="null"
}

show_flags(){
    echo "Flags for PHA Commands:"
}

#cd ${PHA_HOME}

while [[ $# -gt 0 ]]; do
    if [[ ! -n "${PHA_COMMAND}" ]]; then
        PHA_COMMAND=$1
        shift 1
        continue 2
    fi

    case "$1" in
        -a|--addon)
            if [[ -n "$2" ]]; then
                ADDON_COMMAND="$2"
                shift
            else
                echo "Error: -a|--addon requires a value."
                exit 1
            fi
            ;;
        -b|--compfile)
            if [[ -n "$2" ]]; then
                COM_FILE="$2"
                shift
            else
                echo "Error: -b|--composefile requires a value."
                exit 1
            fi
            ;;
        -c|--contname)
            if [[ -n "$2" ]]; then
                CONT_NAME="$2"
                shift
            else
                echo "Error: -c|--contname requires a value."
                exit 1
            fi
            ;;
        -d|--uid)
            if [[ -n "$2" ]]; then
                UID_VAR="$2"
                shift
            else
                echo "Error: -d|--uid requires a value."
                exit 1
            fi
            ;;
        -e|--env)
            if [[ -n "$2" ]]; then
                ENV_FILE="$2"
                shift
            else
                echo "Error: -e|--env requires a value."
                exit 1
            fi
            ;;
        -f|--dockerfile)
            if [[ -n "$2" ]]; then
                DOCKERFILE_PATH="$2"
                shift
            else
                echo "Error: -f|--dockerfile requires a value."
                exit 1
            fi
            ;;
        -g|--gid)
            if [[ -n "$2" ]]; then
                GID_VAR="$2"
                shift
            else
                echo "Error: -g|--gid requires a value."
                exit 1
            fi
            ;;
        -h|--help)
            show_command_help
            ;;
        -i|--image)
            if [[ -n "$2" ]]; then
                DOC_IMG="$2"
                shift
            else
                echo "Error: -i|--image requires a value."
                exit 1
            fi
            ;;
        -o|--compcmd)
            if [[ -n "$2" ]]; then
                COM_COMMAND="$2"
                shift
            else
                echo "Error: -o|--compcmd requires a value."
                exit 1
            fi
            ;;
        -p|--pythonfile)
            if [[ -n "$2" ]]; then
                PYTHONFILE_NAME="$2"
                shift
            else
                echo "Error: -p|--pythonfile requires a value."
                exit 1
            fi
            ;;
        -q|--aptgetfile)
            if [[ -n "$2" ]]; then
                APTGETFILE_NAME="$2"
                shift
            else
                echo "Error: -q|--aptgetfile requires a value."
                exit 1
            fi
            ;;
        -r|--scriptfile)
            if [[ -n "$2" ]]; then
                SCRIPTFILE_NAME="$2"
                shift
            else
                echo "Error: -r|--scriptfile requires a value."
                exit 1
            fi
            ;;
        -t|--tag)
            if [[ -n "$2" ]]; then
                DOC_TAG="$2"
                shift
            else
                echo "Error: -t|--tag requires a value."
                exit 1
            fi
            ;;
        *)
        echo "Unknown flag: $1"
        echo ""
        show_flags
        exit 1
        ;;
    esac
    shift
done


case "${PHA_COMMAND}" in
    -h|--help)
        show_help
        ;;
    active)
        docker ps ${ADDON_COMMAND}
        ;;
    build)
        ${PHA_HOME}/build.sh -d ${UID_VAR} \
                                -e ${ENV_FILE} \
                                -g ${GID_VAR}
        ;;
    build-file)
        ${PHA_HOME}/docker_scripts/build-file.sh -f ${DOCKERFILE_PATH} \
                                                    -i ${DOC_IMG} \
                                                    -p ${PYTHONFILE_NAME} \
                                                    -q ${APTGETFILE_NAME} \
                                                    -r ${SCRIPTFILE_NAME} \
                                                    -t ${DOC_TAG}
        ;;
    compose)
        ${PHA_HOME}/docker_share/run-compose.sh -b ${COM_FILE} \
                                                -d ${UID_VAR} \
                                                -e ${ENV_FILE} \
                                                -g ${GID_VAR} \
                                                -o ${COM_COMMAND}
        ;;
    containers)
        docker ps -a ${ADDON_COMMAND}
        ;;
    enter)
        ${PHA_HOME}/docker_share/enter.sh -c ${CONT_NAME}\
                                            -e ${ENV_FILE}
        ;;
    images)
        docker images
        ;;
    null)
        ;;
    run)
        ${PHA_HOME}/docker_scripts/run-env.sh -d ${UID_VAR} \
                                            -e ${ENV_FILE} \
                                            -g ${GID_VAR}
        ;;
    run-cpu)
        ${PHA_HOME}/docker_scripts/run-cpu-env.sh -d ${UID_VAR} \
                                                -e ${ENV_FILE} \
                                                -g ${GID_VAR}
        ;;
    *)
        echo "Unknown option: $1"
        echo ""
        show_help
        exit 1
        ;;
esac

#cd ${CURRENT_DIR}