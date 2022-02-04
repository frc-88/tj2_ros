#!/usr/bin/env bash

LOGS_DIR=/media/storage/bags
LATEST_BAG=`find $LOGS_DIR -type f -name *.bag -printf '%T@ %p\n' | sort -n | tail -1 | cut -f2- -d" "`
echo $LATEST_BAG
cd ~/tj2_ros/tj2_tools/tj2_tools/rosbag_to_file
python3 convert.py -a -o /media/storage/bags --path $LATEST_BAG
