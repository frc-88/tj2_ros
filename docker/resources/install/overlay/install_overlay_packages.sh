#!/bin/bash

set -e

cd ${DEP_ROS_WS_SRC}
wstool merge -t . /root/install/overlay/tj2_ros_overlay.rosinstall
wstool update -t .

cd ${DEP_ROS_WS_ROOT}
/root/install/rosdep_install.sh
apt-get clean
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF
