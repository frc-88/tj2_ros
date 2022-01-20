#!/usr/bin/env bash

LOGS_DIR=/media/storage/bags
LATEST_BAG=`find $LOGS_DIR -type f -name *.bag -printf '%T@ %p\n' | sort -n | tail -1 | cut -f2- -d" "`
echo $LATEST_BAG
rosrun rosbag_to_file convert.py -a -o /media/storage/bags --path $LATEST_BAG
