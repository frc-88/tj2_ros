#!/bin/bash

set -e

sudo chown -R 1000:1000 /tmp

# tbb
cd /tmp
git clone https://github.com/wjakob/tbb.git
cd tbb
git checkout 9e219e24fe223b299783200f217e9d27790a87b0
cd /tmp/tbb/build
cmake ..
make -j4
sudo make install

cd /tmp
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
git checkout 3e8e974d0d8d6ab318abf56d87506d15d7f2cc35
mkdir build
cd /tmp/apriltag/build
cmake ..
make -j4
sudo make install

# orocos_kinematics_dynamics
cd /tmp
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics
git checkout 5541147d4a220cab97d0ae1efa1aa860557d5c32
git submodule update --init
cd orocos_kdl
mkdir build
cd /tmp/orocos_kinematics_dynamics/orocos_kdl/build
cmake ..
make -j4
sudo make install

# python_orocos_kdl
mkdir -p mkdir ../../python_orocos_kdl/build || true
cd /tmp/orocos_kinematics_dynamics/python_orocos_kdl/build
cmake -D PYTHON_EXECUTABLE=/usr/bin/python3 \
    -D PYTHON_INCLUDE_DIR=/usr/include/python3.6 \
    -D PYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so \
    -D PYBIND11_PYTHON_VERSION=3 ..
make -j4
sudo make install

# LibDS
cd /tmp
git clone https://github.com/FRC-Utilities/LibDS.git
cd LibDS
git checkout 14cac0a7f3b911b3f1c661c3b5f455522ae6638b
qmake -qt=qt5
make -j4
sudo make install

# CmakeWpilib
cd /tmp
git clone --recursive https://github.com/ThadHouse/CmakeWpilib.git
cd CmakeWpilib
git checkout d5bbeb949de35a4c576838e598493779bcf6a328
mkdir build
cd /tmp/CmakeWpilib/build
cmake .. -DWITHOUT_CSCORE=ON -DWITHOUT_JAVA=ON
make -j4
sudo make install

# g2o
cd /tmp
git clone https://github.com/RainerKuemmerle/g2o.git
cd /tmp/g2o
git checkout 20201223_git
mkdir build
cd /tmp/g2o/build
cmake ..
make -j4
sudo make install

# clean up
sudo ldconfig
rm -r /tmp/*

echo "Built and installed all basic libraries"
