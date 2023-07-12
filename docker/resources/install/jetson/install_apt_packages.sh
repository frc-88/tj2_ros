#!/bin/bash

set -e

sudo apt-get update
sudo apt-get install -y \
    libbullet-dev \
    libeigen3-dev \
    libsdl-image1.2-dev \
    libsdl-dev \
    libyaml-cpp-dev \
    libpcl-dev  \
    libvtk6-qt-dev  \
    libspnav-dev  \
    joystick  \
    liborocos-kdl-dev  \
    liborocos-kdl1.3  \
    libnetpbm10-dev  \
    libogg-dev  \
    libtheora-dev  \
    graphviz  \
    libgeographic-dev  \
    python3-pyproj  \
    libavutil-dev  \
    libswscale-dev  \
    v4l-utils  \
    liburdfdom-headers-dev  \
    libtinyxml-dev  \
    liburdfdom-dev  \
    hddtemp  \
    lm-sensors  \
    python3-psutil  \
    libusb-1.0-0*  \
    libsuitesparse-dev \
    python3-termcolor \
    i2c-tools \
    python3-smbus \
    libceres-dev \
    libturbojpeg0-dev \
    python3-twisted \
    python3-tornado \
    python3-autobahn \
    python3-bson \
    libv4l-dev \
    libgeos-dev \
    libturbojpeg-dev

# ffmpeg dependencies
sudo apt-get -y install \
    autoconf \
    automake \
    build-essential \
    cmake \
    git-core \
    libass-dev \
    libfreetype6-dev \
    libgnutls28-dev \
    libmp3lame-dev \
    libsdl2-dev \
    libtool \
    libva-dev \
    libvdpau-dev \
    libvorbis-dev \
    libxcb1-dev \
    libxcb-shm0-dev \
    libxcb-xfixes0-dev \
    ninja-build \
    pkg-config \
    texinfo \
    wget \
    yasm \
    zlib1g-dev \
    libunistring-dev \
    nasm \
    libx264-dev \
    libx265-dev \
    libnuma-dev \
    libvpx-dev \
    libfdk-aac-dev \
    libopus-dev


sudo apt-get upgrade -y
sudo apt-get autoremove -y

echo "Installed all basic apt packages"
