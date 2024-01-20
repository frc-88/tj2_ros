#!/bin/bash

set -e

ln -s /opt/tj2/tj2_ros ${HOME}/tj2_ros

sudo rm -r /usr/local/zed/resources
sudo rm -r /usr/local/zed/settings
ln -s /opt/tj2/tj2_ros/src/tj2_data/data/zed/resources /usr/local/zed/resources
ln -s /opt/tj2/tj2_ros/src/tj2_data/data/zed/settings /usr/local/zed/settings
sudo chown -R 1000:1000 /opt/tj2
sudo chown -R 1000:1000 /usr/local/zed/
