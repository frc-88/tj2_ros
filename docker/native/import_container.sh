#!/bin/bash
set -e

if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

cd /media/storage/docker

# echo "Importing tj2_ros image. This will take a while."
# docker load -i tj2_ros.tar.gz
echo "Decompressing docker_ros_ws"
tar -xvf tj2_ros_ws.tar.gz
echo "Done!"
