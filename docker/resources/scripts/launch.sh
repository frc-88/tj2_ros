#!/bin/bash
echo "Starting tj2_ros"

SESSION=tj2_ros

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
fi

tmux send -t $SESSION "source /root/scripts/enable_tmux_logger.sh tj2_ros" ENTER
tmux send -t $SESSION "source /opt/ros/${ROS_DISTRO}/setup.bash" ENTER
tmux send -t $SESSION "source ${HOME}/ros_ws/devel/setup.bash" ENTER
tmux send -t $SESSION "source ${HOME}/ros_ws/src/tj2_ros/scripts/startup.sh" ENTER
tmux send -t $SESSION "roslaunch --wait tj2_bringup tj2_bringup.launch --screen" ENTER

sleep 2
/root/ros_ws/src/tj2_ros/scripts/tail-session.sh

# sleep infinity
# echo "Starting tj2_ros"
# source /opt/ros/${ROS_DISTRO}/setup.bash
# source ${HOME}/ros_ws/devel/setup.bash
# source ${HOME}/ros_ws/src/tj2_ros/scripts/startup.sh
# echo "Robot: ${ROBOT}"
# roslaunch --wait tj2_bringup tj2_bringup.launch --screen
