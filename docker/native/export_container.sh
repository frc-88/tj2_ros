#!/bin/bash
set -e

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

echo "Exporting tj2_ros image. This will take a very long time."
mkdir -p /media/storage/docker
docker save tj2_ros:latest | gzip > /media/storage/docker/tj2_ros.tar.gz
echo "Compressing docker_ros_ws"
tar -cvf /media/storage/tj2_ros_ws.tar.gz /media/storage/docker_ros_ws
echo "Done!"
