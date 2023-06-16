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

# libaom
cd /tmp
git -C aom pull 2> /dev/null || git clone --depth 1 https://aomedia.googlesource.com/aom
cd aom
mkdir -p build
cd build
cmake -DENABLE_TESTS=OFF -DENABLE_NASM=on ..
make -j4
sudo make install

# libsvtav1
cd /tmp
git -C SVT-AV1 pull 2> /dev/null || git clone https://gitlab.com/AOMediaCodec/SVT-AV1.git
cd SVT-AV1
git checkout v0.9.1
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_DEC=OFF -DBUILD_SHARED_LIBS=OFF ..
make -j4
sudo make install

# dav1d
cd /tmp
git clone https://code.videolan.org/videolan/dav1d.git
cd dav1d
git checkout 0.5.2
mkdir build
cd build
meson --bindir="/usr/local/bin" ..
ninja
sudo ninja install

# ffmpeg
cd /tmp
git clone https://github.com/FFmpeg/FFmpeg.git
cd FFmpeg
git checkout n6.0

./configure \
  --enable-gpl \
  --enable-gnutls \
  --enable-libaom \
  --enable-libass \
  --enable-libfdk-aac \
  --enable-libfreetype \
  --enable-libmp3lame \
  --enable-libopus \
  --enable-libsvtav1 \
  --enable-libdav1d \
  --enable-libvorbis \
  --enable-libvpx \
  --enable-libx264 \
  --enable-libx265 \
  --enable-nonfree \
  --enable-shared \
  --extra-libs="-lpthread"

make -j4
sudo make install && hash -r

# clean up
sudo ldconfig
rm -r /tmp/*

echo "Built and installed all basic libraries"
