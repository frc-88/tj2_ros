#!/bin/bash
sudo systemctl status roscore.service | grep Active -B2
sudo systemctl status roslaunch.service | grep Active -B2
rosnode list
