#!/bin/bash
WHITELIST_PACKAGES=$1
BLACKLIST_PACKAGES=$2

echo "ros workspace src: ${ROS_WS_SRC}"

cd ${ROS_WS_SRC}
if [ -z $WHITELIST_PACKAGES ] && [ -z $BLACKLIST_PACKAGES ]; then
    /root/scripts/clone_ros_packages.sh    
    if [ ! -f ${ROS_WS_SRC}/PATCH_APPLIED ]; then
        /root/scripts/apply_patches.sh ${ROS_WS_SRC}
        touch ${ROS_WS_SRC}/PATCH_APPLIED
    fi

    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd ${ROS_WS_ROOT}
    rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r || true
else
    echo "whitelist packages: ${WHITELIST_PACKAGES}"
    echo "blacklist packages: ${BLACKLIST_PACKAGES}"

fi

source /opt/ros/${ROS_DISTRO}/setup.bash
cd ${ROS_WS_ROOT}
catkin_make -DCATKIN_WHITELIST_PACKAGES="$WHITELIST_PACKAGES" -DCATKIN_BLACKLIST_PACKAGES="$BLACKLIST_PACKAGES"
