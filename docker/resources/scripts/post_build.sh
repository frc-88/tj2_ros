#!/bin/bash
echo "ros workspace src: ${ROS_WS_SRC}"
cd ${ROS_WS_SRC}
/root/scripts/clone_ros_packages.sh
/root/scripts/apply_patches.sh ${ROS_WS_SRC}
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ${ROS_WS_ROOT}
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r || true
catkin_make
/root/scripts/install_tools.sh
