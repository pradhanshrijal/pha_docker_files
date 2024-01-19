#!/bin/bash
ENV_VERSION=$(cat ENV_VERSION)
docker build -t pha-envs/jekyll-22:${ENV_VERSION} -f Dockerfile --no-cache .