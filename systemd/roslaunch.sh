#!/usr/bin/env bash
source /home/${USER}/packages_ros_ws/devel/setup.bash
source /home/${USER}/noetic_ws/install_isolated/setup.bash
source /home/${USER}/ros_ws/devel/setup.bash
source /usr/local/bin/env.sh

export ROS_HOME=/home/${USER}/.ros
export DISPLAY=:0
roslaunch tj2_bringup tj2_bringup.launch --wait &
PID=$!
wait "$PID"
