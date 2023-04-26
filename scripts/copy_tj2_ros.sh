#!/bin/bash

set -e

sudo chown 1000:1000 ${ROS_WS_ROOT}
mkdir -p ${ROS_WS_SRC}
rsync -av ${HOME}/tj2_ros/src/* ${ROS_WS_SRC}/tj2_ros
