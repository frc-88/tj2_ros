#!/bin/bash

RM_PATTERN="$1"

if [ -z "${RM_PATTERN}" ]; then
    echo "Removal pattern is empty! Exiting."
    exit 1
fi

rm -r ${ROS_WS_SRC}/tj2_ros/tj2_data/data/"${RM_PATTERN}"
rm -r /opt/tj2/tj2_ros/src/tj2_data/data/"${RM_PATTERN}"
