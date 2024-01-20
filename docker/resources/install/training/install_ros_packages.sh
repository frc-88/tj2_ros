#!/bin/bash

set -e

cd ${DEP_ROS_WS_SRC}/zed-ros-wrapper
git checkout -f  # restore deleted ros wrapper

cd ${DEP_ROS_WS_ROOT}
/opt/tj2/install/rosdep_install.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

echo "Installed ROS training packages"
