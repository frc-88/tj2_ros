#!/bin/bash

set -e

sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-get update
apt-get install -y \
    python3-osrf-pycommon \
    python3-wstool \
    python3-catkin-pkg

mkdir -p ${DEP_ROS_WS_SRC}
cd ${DEP_ROS_WS_SRC}
wstool init .
wstool merge -t . /root/install/evergreen/tj2_ros_evergreen.rosinstall
wstool update -t .
/root/install/evergreen/patch_evergreen_packages.sh

cd ${DEP_ROS_WS_ROOT}
/root/install/rosdep_install.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

echo "Installed ROS evergreen packages"
