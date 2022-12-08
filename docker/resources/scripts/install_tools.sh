#!/bin/bash
cd ${ROS_WS_SRC}/tj2_ros/tj2_tools
python3 setup.py install --user --prefix=${SHARED_TOOLS_INSTALL}
