#!/bin/bash
set -e

sudo apt-get update
sudo apt-get install rsync -y
rm -r /tmp/workstation_tj2_ros/sys
sudo rsync -rtul --ignore-existing /tmp/workstation_tj2_ros/* /
rm -r /tmp/*
