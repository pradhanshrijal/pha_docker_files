#!/bin/bash
ENV_VERSION=$(cat ENV_VERSION)
docker build -t pha-envs/carla-22-mini:${ENV_VERSION} -f Dockerfile --no-cache .