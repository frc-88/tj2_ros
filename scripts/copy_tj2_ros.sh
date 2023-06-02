#!/bin/bash

set -e

sudo chown 1000:1000 ${ROS_WS_ROOT}
mkdir -p ${ROS_WS_SRC}
rsync -av --delete --exclude-from=/opt/tj2/tj2_ros/scripts/copy_exclude.txt /opt/tj2/tj2_ros/src/ ${ROS_WS_SRC}/tj2_ros
rsync -rtuv /opt/tj2/tj2_ros/src/tj2_data/data/ ${ROS_WS_SRC}/tj2_ros/tj2_data/data
rsync -rtuv ${ROS_WS_SRC}/tj2_ros/tj2_data/data/ /opt/tj2/tj2_ros/src/tj2_data/data
