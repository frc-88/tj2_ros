#!/usr/bin/env bash
OUTPUT_PREFIX=/media/storage/bags/diffyjr
# OUTPUT_PREFIX=/home/tj2/Diff-Swerve-ROS/tj2_laser_slam/bags
rosrun rosbag_snapshot snapshot -t -o $OUTPUT_PREFIX

