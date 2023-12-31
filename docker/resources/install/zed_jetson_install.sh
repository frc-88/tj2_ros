#!/bin/bash

set -e

cd /opt/tj2
sudo apt-get update
sudo apt-get install --no-install-recommends lsb-release less udev sudo apt-transport-https -y
sudo sh -c 'echo "# R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}" > /etc/nv_tegra_release'
wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}/jetsons
bash ./ZED_SDK_Linux.run silent skip_tools
sudo rm -rf /usr/local/zed/resources/*
sudo rm -rf ZED_SDK_Linux.run
sudo rm -rf /var/lib/apt/lists/*
# This symbolic link is needed to use the streaming features on Jetson inside a container
sudo ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

echo "ZED installation script complete"
