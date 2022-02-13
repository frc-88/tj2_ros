#!/usr/bin/env bash

LOGS_DIR=/media/storage/bags
CONVERT_BAG=$1
if [ -z ${CONVERT_BAG} ]; then
    LATEST_BAG=`find $LOGS_DIR -type f -name *.bag -printf '%T@ %p\n' | sort -n | tail -1 | cut -f2- -d" "`
    echo $LATEST_BAG
    CONVERT_BAG=LATEST_BAG
else
    CONVERT_BAG=$LOGS_DIR/$CONVERT_BAG
fi
cd ~/tj2_ros/tj2_tools/tj2_tools/rosbag_to_file
python3 convert.py -a -o /media/storage/bags --path $CONVERT_BAG
