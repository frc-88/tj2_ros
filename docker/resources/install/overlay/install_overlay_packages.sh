#!/bin/bash

set -e

sudo apt-get update

cd ${DEP_ROS_WS_SRC}
wstool merge -t . /opt/tj2/install/overlay/tj2_ros_overlay.rosinstall
wstool update -t .

cd ${DEP_ROS_WS_ROOT}
/opt/tj2/install/rosdep_install.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

echo "Installed ROS overlay packages"
