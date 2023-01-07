#!/bin/bash
echo "Starting tj2_ros"

SESSION=tj2_ros

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
fi

if [ ! -f /root/roslaunch_pid ]; then
    echo 0 > /root/roslaunch_pid
fi
# SIGINT-handler
int_handler() {
    echo "Caught stop signal"
    pid=`cat /root/roslaunch_pid`
    if [ $pid -ne 0 ]; then
        echo "Stopping roslaunch"
        kill -SIGINT $pid
        tail --pid=$pid -f /dev/null
    fi
    exit 0;
}
trap 'int_handler' SIGINT

tmux send -t $SESSION 'source ${HOME}/install/enable_tmux_logger.sh bw' ENTER
tmux send -t $SESSION 'source /opt/ros/${ROS_DISTRO}/setup.bash' ENTER
tmux send -t $SESSION 'source ${HOME}/ros_ws/devel/setup.bash' ENTER
tmux send -t $SESSION 'source ${HOME}/scripts/startup.sh' ENTER
tmux send -t $SESSION 'roslaunch --wait tj2_bringup tj2_bringup.launch --screen &' ENTER
tmux send -t $SESSION 'echo $! > /root/roslaunch_pid' ENTER

sleep 2
${HOME}/scripts/tail-session.sh
sleep infinity