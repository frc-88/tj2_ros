#!/bin/bash

set -e

apt-get update
apt-get install -y llvm-7*
ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin

rm /usr/bin/python
ln -s /usr/bin/python3 /usr/bin/python

python -m pip install --upgrade pip setuptools
python -m pip install scipy==1.5.4 \
    shapely==1.6.4 \
    dataclasses \
    pynetworktables==2021.0.0 \
    pynmcli==1.0.5 \
    flask==2.0.3 \
    py-trees==0.7.6 \
    open3d==0.15.1 \
    psutil \
    v4l2-fix

python -m pip install Cython
python -m pip install llvmlite==0.32.0
python -m pip install numba==0.49.0

chown ${USER} -R /usr/local/lib/python3.6/dist-packages/

echo "Installed python dependencies"
