#!/bin/bash

set -e

cd /tmp
apt-get update
apt-get install -y build-essential libtool autoconf zip unzip wget zstd
wget -O cmake.sh https://cmake.org/files/v3.23/cmake-3.23.1-linux-aarch64.sh
mkdir /opt/cmake
sh cmake.sh --prefix=/opt/cmake --exclude-subdir --skip-license
mv /usr/bin/cmake /usr/bin/cmake-old
ln -s /opt/cmake/bin/cmake /usr/bin/cmake

echo "CMake upgrade script complete"

