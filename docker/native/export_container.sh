#!/bin/bash
set -e

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

# echo "Exporting tj2_ros image. This will take a very long time."
# mkdir -p /media/storage/docker
# docker save tj2_ros:latest | gzip > /media/storage/docker/tj2_ros.tar.gz
echo "Compressing docker_ros_ws"
cd /media/storage
tar -cvf tj2_ros_ws.tar.gz docker_ros_ws
echo "Done!"
