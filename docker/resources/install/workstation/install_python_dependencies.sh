#!/bin/bash

set -e

sudo apt-get update
sudo apt-get install -y llvm-7*
sudo ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin

sudo rm /usr/bin/python || true
sudo ln -s /usr/bin/python3 /usr/bin/python

python -m pip install --no-cache-dir --upgrade pip setuptools
python -m pip install --no-cache-dir scipy==1.5.4 \
    numpy==1.19.5 \
    matplotlib==3.4.3 \
    shapely==1.6.4 \
    dataclasses \
    pynetworktables==2021.0.0 \
    flask==2.0.3 \
    py-trees==0.7.6 \
    psutil \
    tqdm \
    v4l2-fix \
    jsonlines \
    ezdxf \
    mypy \
    pynput \
    filterpy

python -m pip install Cython --no-cache-dir
python -m pip install llvmlite==0.32.0 --no-cache-dir
python -m pip install numba==0.49.0 --no-cache-dir

echo "Installed python dependencies"
