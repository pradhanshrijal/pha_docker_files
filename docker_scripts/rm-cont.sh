#!/bin/bash

CONT_NAME="pha-22"

while getopts c: flag
do
    case "${flag}" in
        c) CONT_NAME=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

docker rm ${CONT_NAME}