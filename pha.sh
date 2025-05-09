#!/bin/bash

source ${PHA_HOME}/docker_scripts/pha_command_help.sh

DOC_IMG="phaenvs/pha-22"
DOC_TAG="sample"
DOCKERFILE_PATH="envs/pha-22/Dockerfile"
PYTHONFILE_NAME="python_requirements.txt"
APTGETFILE_NAME="apt-get_requirements.txt"
SCRIPTFILE_NAME="script_requirements.sh"

INITSCRIPT_NAME=""
ENV_FILE="${PHA_HOME}/envs/pha-22/pha-22.env"
COM_FILE="docker_scripts/docker-compose.yaml"
COM_COMMAND="up -d"

UID_VAR=$(id -u)
GID_VAR=$(id -g)

ADDON_COMMAND=""
INIT_ADDON="pha_init.sh"
PHA_COMMAND=""
CURRENT_DIR="${PWD}"

show_help(){
    echo "Commands for PHA:

    active:             list all active contianers
    addons:             change to addons folder
    build:              build based on environmental files
    build-file:         build a Dockerfile with scripts
    compose:            user compose-file for pha setup
    containers:         list all containers
    enter:              start and execute an available container
    exec:               execute an active docker container
    flags:              show list of flags for PHA
    home:               change to the home folder
    images:             list all available docker images
    init:               initialize a PHA component
    prune:              docker clean-up
    purge:              remove all docker containers and images
    purge-cont:         remove all docker containers
    purge-imgs:         remove all docker images
    restart:            restart a container
    rm:                 remove a stopped container
    rm-image:           remove an image
    ros-pkgs:           change to ros packages folder
    run:                run a container from environment files
    run-cpu:            run a cpu container from env files
    script:             run initialization script for PHA
    simulators:         change to simulators folder
    ssi:                change to the shared folder
    start:              start an available container
    stop:               stop an active container

Helper Flags for PHA:
    -h | --help         show this information page
    -l | --list         show the list of PHA components
    -v | --version      show the version number of PHA
    
For command instructions: pha {COMMAND} -h"
}

show_command_help()
{
    case "${PHA_COMMAND}" in
    active)
        show_active_help
        exit 1
        ;;
    build)
        show_build_help
        exit 1
        ;;
    build-file)
        show_build_file_help
        exit 1
        ;;
    compose)
        show_compose_help
        exit 1
        ;;
    containers)
        show_containers_help
        exit 1
        ;;
    enter)
        show_enter_help
        exit 1
        ;;
    exec)
        show_exec_help
        exit 1
        ;;
    images)
        show_images_help
        exit 1
        ;;
    init)
        show_init_help
        exit 1
        ;;
    prune)
        show_prune_help
        exit 1
        ;;
    purge)
        show_purge_help
        exit 1
        ;;
    purge-cont)
        show_purge_cont_help
        exit 1
        ;;
    purge-imgs)
        show_purge_imgs_help
        exit 1
        ;;
    restart)
        show_restart_help
        exit 1
        ;;
    rm)
        show_rm_help
        exit 1
        ;;
    rm-image)
        show_rm_image_help
        exit 1
        ;;
    run)
        show_run_help
        exit 1
        ;;
    run-cpu)
        show_run_cpu_help
        exit 1
        ;;
    script)
        show_script_help
        exit 1
        ;;
    start)
        show_start_help
        exit 1
        ;;
    stop)
        show_stop_help
        exit 1
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
    echo "Flags for PHA Commands:
    -a | --addon        addon commands i.e. '--size'
    -b | --compfile     compose file based on yaml for installation
    -c | --contname     container name to enter
    -d | --uid          variable for the UID value
    -e | --env          full path to the environment file
    -f | --dockerfile   Dockerfile path in relative to the PHA Project
    -g | --gid          variable for the GID value
    -h | --help         show this information page
    -i | --image        docker image name to be used as base
    -j | --imagename    full docker image name [image:tag]
    -k | --initpath     path to PHA component
    -l | --list         show the list of PHA components
    -o | --compcmd      compose command | up -d | down | start | stop |
    -p | --pythonfile   absolute file path for python list to be installed
    -q | --aptgetfile   absolute file path for apt-get list to be installed
    -r | --scriptfile   absolute file path for script based installation
    -s | --initscript   initialization script for PHA
    -t | --tag          docker tag name to be used for base image
    -v | --version      show the version number of PHA
    "
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
                echo "Error: -a | --addon requires a value."
                exit 1
            fi
            ;;
        -b|--compfile)
            if [[ -n "$2" ]]; then
                COM_FILE="$2"
                shift
            else
                echo "Error: -b | --composefile requires a value."
                exit 1
            fi
            ;;
        -c|--contname)
            if [[ -n "$2" ]]; then
                CONT_NAME="$2"
                shift
            else
                echo "Error: -c | --contname requires a value."
                exit 1
            fi
            ;;
        -d|--uid)
            if [[ -n "$2" ]]; then
                UID_VAR="$2"
                shift
            else
                echo "Error: -d | --uid requires a value."
                exit 1
            fi
            ;;
        -e|--env)
            if [[ -n "$2" ]]; then
                ENV_FILE="$2"
                shift
            else
                echo "Error: -e | --env requires a value."
                exit 1
            fi
            ;;
        -f|--dockerfile)
            if [[ -n "$2" ]]; then
                DOCKERFILE_PATH="$2"
                shift
            else
                echo "Error: -f | --dockerfile requires a value."
                exit 1
            fi
            ;;
        -g|--gid)
            if [[ -n "$2" ]]; then
                GID_VAR="$2"
                shift
            else
                echo "Error: -g | --gid requires a value."
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
                echo "Error: -i | --image requires a value."
                exit 1
            fi
            ;;
        -j|--imagename)
            if [[ -n "$2" ]]; then
                IMG_NAME="$2"
                shift
            else
                echo "Error: -j | --imagename requires a value."
                exit 1
            fi
            ;;
        -k|--initpath)
            if [[ -n "$2" ]]; then
                INIT_ADDON="$2/$INIT_ADDON"
                shift
            else
                echo "Error: -k | --initpath requires a value."
                exit 1
            fi
            ;;
        -o|--compcmd)
            if [[ -n "$2" ]]; then
                COM_COMMAND="$2"
                shift
            else
                echo "Error: -o | --compcmd requires a value."
                exit 1
            fi
            ;;
        -p|--pythonfile)
            if [[ -n "$2" ]]; then
                PYTHONFILE_NAME="$2"
                shift
            else
                echo "Error: -p | --pythonfile requires a value."
                exit 1
            fi
            ;;
        -q|--aptgetfile)
            if [[ -n "$2" ]]; then
                APTGETFILE_NAME="$2"
                shift
            else
                echo "Error: -q | --aptgetfile requires a value."
                exit 1
            fi
            ;;
        -r|--scriptfile)
            if [[ -n "$2" ]]; then
                SCRIPTFILE_NAME="$2"
                shift
            else
                echo "Error: -r | --scriptfile requires a value."
                exit 1
            fi
            ;;
        -s|--initscript)
            if [[ -n "$2" ]]; then
                INITSCRIPT_NAME="$2"
                shift
            else
                echo "Error: -s | --initscript requires a value."
                exit 1
            fi
            ;;
        -t|--tag)
            if [[ -n "$2" ]]; then
                DOC_TAG="$2"
                shift
            else
                echo "Error: -t | --tag requires a value."
                exit 1
            fi
            ;;
        *)
        echo "Unknown flag: $1"
        echo "Set as input"
        IMG_NAME="$1"
        CONT_NAME="$1"
        INIT_ADDON="$1/$INIT_ADDON"
        #show_flags
        #exit 1
        ;;
    esac
    shift
done


case "${PHA_COMMAND}" in
    -h|--help)
        show_help
        ;;
    -l|--list)
        printenv | grep PHA
        ;;
    -v|--version)
        echo ${PHA_VERSION}
        ;;
    active)
        docker ps ${ADDON_COMMAND}
        ;;
    addons)
        cd ${ADDONS_PHA}
        exec bash
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
        ${PHA_HOME}/docker_scripts/run-compose.sh -b ${COM_FILE} \
                                                -d ${UID_VAR} \
                                                -e ${ENV_FILE} \
                                                -g ${GID_VAR} \
                                                -o ${COM_COMMAND}
        ;;
    containers)
        docker ps -a ${ADDON_COMMAND}
        ;;
    enter)
        ${PHA_HOME}/docker_scripts/enter.sh -c ${CONT_NAME}
        ;;
    exec)
        ${PHA_HOME}/docker_scripts/exec-cont.sh -c ${CONT_NAME}
        ;;
    flags)
        show_flags
        ;;
    home)
        cd ${PHA_HOME}
        exec bash
        echo ${PHA_HOME}
        ;;
    images)
        docker images
        ;;
    init)
        ${INIT_ADDON}
        ;;
    prune)
        docker system prune
        ;;
    purge)
        docker rm -f $(docker ps -a -q)
        docker image remove -f $(docker images -a -q)
        ;;
    purge-cont)
        docker rm -f $(docker ps -a -q)
        ;;
    purge-imgs)
        docker rm -f $(docker ps -a -q)
        docker image remove -f $(docker images -a -q)
        ;;
    restart)
        docker restart ${CONT_NAME}
        ;;
    rm)
        docker rm ${CONT_NAME}
        ;;
    rm-image)
        docker image rm ${IMG_NAME}
        ;;
    ros-pkgs)
        cd ${ROS_PHA}
        exec bash
        echo ${ROS_PHA}
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
    script)
        ${INITSCRIPT_NAME}
        ;;
    simulators)
        cd ${SIMULATORS_PHA}
        exec bash
        echo ${SIMULATORS_PHA}
        ;;
    ssi)
        cd ${SSI_PATH}
        exec bash
        echo ${SSI_PATH}
        ;;
    start)
        docker start ${CONT_NAME}
        ;;
    stop)
        docker stop ${CONT_NAME}
        ;;
    *)
        echo "Unknown option: $1"
        echo ""
        show_help
        exit 1
        ;;
esac

#cd ${CURRENT_DIR}