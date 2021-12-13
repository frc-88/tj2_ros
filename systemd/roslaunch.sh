#!/usr/bin/env bash
echo "Starting tj2 roslaunch"

SESSION=roslaunch

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
fi

tmux send -t $SESSION "source /home/${USER}/packages_ros_ws/devel/setup.bash
source /home/${USER}/noetic_ws/install_isolated/setup.bash
source /home/${USER}/ros_ws/devel/setup.bash
source /usr/local/bin/env.sh

export ROS_HOME=/home/${USER}/.ros
export DISPLAY=:0
roslaunch --wait tj2_bringup tj2_bringup.launch --screen
" Enter
