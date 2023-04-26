#!/bin/bash

mkdir -p ${ROS_WS_SRC}
rsync -av ${HOME}/tj2_ros/src/* ${ROS_WS_SRC}/tj2_ros
