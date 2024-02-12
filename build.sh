#!/bin/bash
# Usage: source build.sh ${DOC_IMGAINER_NAME}
DOC_IMG_NAME=$1
DOC_IMG_NAME="${DOC_IMG_NAME:=pha-22}"
ENV_VERSION=$(cat envs/$DOC_IMG_NAME/ENV_VERSION)
docker build -t phaenvs/${DOC_IMG_NAME}:${ENV_VERSION} -f envs/$DOC_IMG_NAME/Dockerfile --no-cache .