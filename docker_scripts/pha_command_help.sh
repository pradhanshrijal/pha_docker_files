#!/bin/bash

show_active_help()
{
    echo "usage: pha active -a [addons]

List all active docker containers.

options:
    -a | --addon        addon commands i.e. '--size'

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_build_help()
{
    echo "usage: pha build -d [UID_VAR] -e [ENV_FILE] -g [GID_VAR]

Build docker images based on an environment file.

options:
    -d | --uid          variable for the UID value
    -e | --env          absolute path to the environment file
    -g | --gid          variable for the GID value

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_build_file_help()
{
    echo "usage: pha build-file -f [DOC_FILE] -i [DOC_IMG] -p [PYTHON] -q [APTGET] -r [SCRIPTFILE] -t [DOC_TAG]

Build docker images based on installations files.

options:
    -f | --dockerfile   Dockerfile path in relative to the PHA Project
    -i | --image        docker image name to be used as base
    -p | --pythonfile   absolute file path for python list to be installed
    -q | --aptgetfile   absolute file path for apt-get list to be installed
    -r | --scriptfile   absolute file path for script based installation
    -t | --tag          docker tag name to be used for base image

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_compose_help()
{
    echo "usage: pha compose -b [COM_FILE] -d [UID_VAR] -e [ENV_FILE] -g [GID_VAR]

Build docker images based on compose file and environment file.

options:
    -b | --compfile     compose file based on yaml for installation
    -d | --uid          variable for the UID value
    -e | --env          full path to the environment file
    -g | --gid          variable for the GID value
    -o | --compcmd      compose command | up -d | down | start | stop |

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_containers_help()
{
    echo "usage: pha containers -a [addons]

List all docker containers.

options:
    -a | --addon        addon commands i.e. '--size'

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_enter_help()
{
    echo "usage: pha enter -c [CONT_NAME]

Start and execute an available container.

options:
    -c | --contname     container name to enter

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_exec_help()
{
    echo "usage: pha exec -c [CONT_NAME]

Execute an active container.

options:
    -c | --contname     container name to execute

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_images_help()
{
    echo "usage: pha images

List all available images.

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_prune_help()
{
    echo "usage: pha prune

Clean-up. Remove all stopped docker containers and dangling images.

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_purge_help()
{
    echo "usage: pha purge

Remove all docker containers and images.

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_purge_cont_help()
{
    echo "usage: pha purge-cont

Remove all docker containers.

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_purge_imgs_help()
{
    echo "usage: pha purge-imgs

Remove all docker images and associated containers.

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_run_help()
{
    echo "usage: pha run -d [UID_VAR] -e [ENV_FILE] -g [GID_VAR]

Run docker container based on an environment file.

options:
    -d | --uid          variable for the UID value
    -e | --env          absolute path to the environment file
    -g | --gid          variable for the GID value

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_run_cpu_help()
{
    echo "usage: pha run-cpu -d [UID_VAR] -e [ENV_FILE] -g [GID_VAR]

Run cpu docker container based on an environment file.

options:
    -d | --uid          variable for the UID value
    -e | --env          absolute path to the environment file
    -g | --gid          variable for the GID value

Helper Flags for PHA:
    -h | --help         show this information page"
}

show_start_help()
{
    echo "usage: pha start -c [CONT_NAME]

Start a container.

options:
    -c | --contname     container name to execute

Helper Flags for PHA:
    -h | --help         show this information page"
}