#!/bin/bash

set -e

# LibDS
cd /tmp
git clone https://github.com/FRC-Utilities/LibDS.git
cd /tmp/LibDS
qmake -qt=qt5
make -j4
sudo make install

# CmakeWpilib
cd /tmp
git clone --recursive https://github.com/ThadHouse/CmakeWpilib.git
cd CmakeWpilib
mkdir build
cd /tmp
sed -i "16i #define __PTHREAD_SPINS         0, 0" CmakeWpilib/libraries/wpiutil/wpiutil/src/main/native/include/support/priority_mutex.h
cd /tmp/CmakeWpilib/build
cmake .. -DWITHOUT_CSCORE=ON -DWITHOUT_JAVA=ON
make -j4
sudo make install

# clean up
sudo ldconfig
rm -r /tmp/*

echo "Built and installed all basic libraries"
