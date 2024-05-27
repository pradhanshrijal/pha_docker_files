#!/bin/bash

IN_CUDA_VERSION_NUMBER=$1
IN_CUDNN_VERSION=$2
IN_CUDNN_PACKAGE_NAME=$3
IN_CUDA_VERSION_NUMBER=${IN_CUDA_VERSION_NUMBER:=11.7}

IN_CUDNN_VERSION=${IN_CUDNN_VERSION:=8.5.0.96}
IN_CUDNN_PACKAGE_NAME=${IN_CUDNN_PACKAGE_NAME:=libcudnn8}

RUN sudo apt install -y --no-install-recommends \
        ${IN_CUDNN_PACKAGE_NAME}=${IN_CUDNN_VERSION}-1+cuda${IN_CUDA_VERSION_NUMBER} \
        ${IN_CUDNN_PACKAGE_NAME}-dev=${IN_CUDNN_VERSION}-1+cuda${IN_CUDA_VERSION_NUMBER}

RUN sudo apt-mark hold ${IN_CUDNN_PACKAGE_NAME}
RUN rm -rf /var/lib/apt/lists/*
