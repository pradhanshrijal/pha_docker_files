#!/bin/bash

IN_UBUNTU=$1
IN_TRT=$2
IN_CUDA=$3
IN_TRT="${IN_TRT:=8.6.1}"
IN_CUDA="${IN_CUDA:=11.8}"
IN_UBUNTU="${IN_UBUNTU:=2204}"

wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/secure/${IN_TRT}/local_repos/nv-tensorrt-local-repo-ubuntu${IN_UBUNTU}-${IN_TRT}-cuda-${IN_CUDA}_1.0-1_amd64.deb
sudo dpkg -i nv-tensorrt-local-repo-ubuntu${IN_UBUNTU}-${IN_TRT}-cuda-${IN_CUDA}_1.0-1_amd64.deb
sudo cp /var/nv-tensorrt-local-repo-ubuntu${IN_UBUNTU}-${IN_TRT}-cuda-${IN_CUDA}/nv-tensorrt-local-3E951519-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt install tensorrt -y
sudo apt install python3-libnvinfer-dev -y
python3 -m pip install protobuf
sudo apt install uff-converter-tf -y
python3 -m pip install onnx
sudo apt install onnx-graphsurgeon -y
rm -rf nv-tensorrt-local-repo-ubuntu${IN_UBUNTU}-${IN_TRT}-cuda-${IN_CUDA}_1.0-1_amd64.deb
sudo rm -rf /etc/apt/sources.list.d/nv-tensorrt-local-ubuntu${IN_UBUNTU}-${IN_TRT}-cuda-${IN_CUDA}.list