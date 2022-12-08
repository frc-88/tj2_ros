
#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${HOME}/ros_ws/install/setup.bash
source ${HOME}/tj2_ros_ws/install/setup.bash
source ${HOME}/tj2_ros_ws/src/tj2_ros/scripts/startup.sh

roslaunch tj2_bringup tj2_bringup.launch --screen
