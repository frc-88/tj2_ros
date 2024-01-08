#!/bin/bash

set -e

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt-get update
sudo apt-get install -y \
    python3-osrf-pycommon \
    python3-wstool \
    python3-catkin-pkg

mkdir -p ${DEP_ROS_WS_SRC}
cd ${DEP_ROS_WS_SRC}
wstool init .
wstool merge -t . /opt/tj2/install/ros/tj2_ros.rosinstall
wstool update -t .
/opt/tj2/install/ros/patch_ros_packages.sh

cd ${DEP_ROS_WS_ROOT}
/opt/tj2/install/rosdep_install.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

echo "Installed ROS packages"
