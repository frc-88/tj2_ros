#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${DEP_ROS_WS_ROOT}/devel/setup.bash
source ${ROS_WS_ROOT}/devel/setup.bash
source /opt/tj2/scripts/startup.sh

exec "$@"
