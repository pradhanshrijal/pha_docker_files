#!/bin/bash
ENV_VERSION=$(cat ENV_VERSION)
docker build -t pha-envs/pha-20:${ENV_VERSION} -f Dockerfile --no-cache .