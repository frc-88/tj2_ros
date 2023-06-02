#!/bin/bash

sudo rm -r /tmp/workstation_tj2_ros/sys
sudo rsync -az /tmp/workstation_tj2_ros/* /
sudo rm -r /tmp/*
