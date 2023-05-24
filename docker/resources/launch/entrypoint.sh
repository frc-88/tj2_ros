#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${DEP_ROS_WS_ROOT}/devel/setup.bash
source ${ROS_WS_ROOT}/devel/setup.bash
source /opt/tj2/scripts/startup.sh

if [ ! -z ${REMOTE_MACHINE} ]; then
    source /opt/tj2/scripts/set_client.sh ${REMOTE_MACHINE}
fi

exec "$@"
