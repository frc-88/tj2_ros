#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${HOME}/ros_ws/devel/setup.bash
source ${HOME}/scripts/startup.sh

exec "$@"
