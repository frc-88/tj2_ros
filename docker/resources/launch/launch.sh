#!/bin/bash
echo "Starting tj2_ros"

SESSION=tj2_ros

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
fi

if [ ! -f /home/tj2/roslaunch_pid ]; then
    echo 0 > /home/tj2/roslaunch_pid
fi
# SIGINT-handler
int_handler() {
    echo "Caught stop signal"
    pid=`cat /home/tj2/roslaunch_pid`
    if [ $pid -ne 0 ]; then
        echo "Stopping roslaunch"
        kill -SIGINT $pid
        tail --pid=$pid -f /dev/null
    fi
    exit 0;
}
trap 'int_handler' SIGINT

tmux send -t $SESSION 'source ${HOME}/install/enable_tmux_logger.sh tj2_ros' ENTER
tmux send -t $SESSION 'source ${HOME}/scripts/set_master.sh ${ROS_MASTER_INTERFACE}' ENTER
tmux send -t $SESSION 'source /opt/ros/${ROS_DISTRO}/setup.bash' ENTER
tmux send -t $SESSION 'source ${DEP_ROS_WS_ROOT}/devel/setup.bash' ENTER
tmux send -t $SESSION 'source ${ROS_WS_ROOT}/devel/setup.bash' ENTER
tmux send -t $SESSION 'source ${HOME}/scripts/startup.sh' ENTER
tmux send -t $SESSION 'roslaunch --wait tj2_bringup tj2_bringup.launch --screen &' ENTER
tmux send -t $SESSION 'echo $! > /home/tj2/roslaunch_pid' ENTER

sleep 2
${HOME}/scripts/tail-session.sh
sleep infinity
