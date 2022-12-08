#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${HOME}/ros_ws/devel/setup.bash
source ${HOME}/ros_ws/src/tj2_ros/scripts/startup.sh
echo "Robot: ${ROBOT}"
roslaunch --wait tj2_bringup tj2_bringup.launch --screen
