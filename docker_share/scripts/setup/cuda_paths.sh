#!/bin/bash

# First input [$1] is the Cuda Folder - ex: cuda-11.3

PATH=/usr/local/$1/bin/:$PATH
CUDA_PATH=/usr/local/$1
CUDA_HOME=/usr/local/$1
LD_LIBRARY_PATH=/usr/local/$1/lib64:$LD_LIBRARY_PATH
CUDA_TOOLKIT_ROOT_DIR=/usr/local/$1
CUDACXX=/usr/local/$1/bin/nvcc
CUDA_INCLUDE_DIRS=/usr/local/$1/include
CUDA_CUDART_LIBRARY=/usr/local/$1/lib64/libcudart.so
CUDA_MODULE_LOADING=LAZY