#!/bin/bash

set -e

sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update
sudo apt-get install -y \
    python3-osrf-pycommon \
    python3-wstool \
    python3-catkin-pkg

mkdir -p ${DEP_ROS_WS_SRC}
cd ${DEP_ROS_WS_SRC}
wstool init .
wstool merge -t . /opt/tj2/install/workstation/tj2_ros_workstation.rosinstall
wstool update -t .

# TODO: install ZED SDK in docker
rm -r ${DEP_ROS_WS_SRC}/zed-ros-wrapper/zed_nodelets

cd ${DEP_ROS_WS_ROOT}
/opt/tj2/install/rosdep_install.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

sudo rm -rf /var/lib/apt/lists/*

echo "Installed ROS workstation packages"
