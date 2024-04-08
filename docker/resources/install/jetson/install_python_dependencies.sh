#!/bin/bash

set -e

sudo apt-get update
sudo apt-get install -y llvm-10* python3-dev
sudo ln -s /usr/lib/llvm-10/bin/llvm-config /usr/bin

sudo rm /usr/bin/python || true
sudo ln -s /usr/bin/python3 /usr/bin/python

python -m pip install --no-cache-dir --upgrade pip setuptools
python -m pip install --no-cache-dir \
    scipy==1.10.1 \
    shapely==2.0.3 \
    dataclasses \
    pynetworktables==2021.0.0 \
    flask==3.0.3 \
    psutil==5.9.8 \
    tqdm==4.66.2 \
    v4l2-fix==0.3

sudo -H python -m pip install meson==1.4.0 --no-cache-dir
python -m pip install Cython --no-cache-dir
python -m pip install llvmlite==0.41.1 --no-cache-dir
python -m pip install numba==0.58.1 --no-cache-dir

echo "Installed python dependencies"
