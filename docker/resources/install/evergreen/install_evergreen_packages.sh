#!/bin/bash

set -e

sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-get update
apt-get install -y \
    python3-osrf-pycommon \
    python3-wstool \
    python3-catkin-pkg

cd ${DEP_ROS_WS_SRC}
wstool init .
wstool merge -t . ${HOME}/install/evergreen/tj2_ros_evergreen.rosinstall
wstool update -t .
${HOME}/install/evergreen/patch_evergreen_packages.sh

${HOME}/install/rosdep_install.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF
