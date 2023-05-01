#!/bin/bash

set -e

apt-get update
apt-get install -y \
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
    ffmpeg  \
    libavcodec-dev  \
    libavformat-dev  \
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
    libswscale-dev \
    libsuitesparse-dev \
    python3-termcolor \
    i2c-tools \
    python3-smbus \
    libceres-dev \
    libturbojpeg0-dev \
    python3-twisted \
    python3-tornado \
    python3-autobahn \
    python3-bson

apt-get upgrade -y

echo "Installed all basic apt packages"
