#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${DEP_ROS_WS_ROOT}/devel/setup.bash

cd ${ROS_WS_SRC}
PACKAGE_LIST=`ls -d tj2_* | sed 's/\///g'`
PACKAGE_LIST=`echo "$PACKAGE_LIST" | tr '\n' ';'`
cd ${ROS_WS_ROOT}
catkin config --extend ${DEP_ROS_WS_ROOT}
catkin config --skiplist tj2_yolo
catkin build
