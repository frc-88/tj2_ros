BASE_DIR=$(realpath "$(dirname $0)")

SESSION=ros
HOST_IP=$1
SSH_KEY_PATH=$2
JOYSTICK_PATH=$3

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d

    tmux split-window -t $SESSION:0
    tmux split-window -t $SESSION:0
    tmux split-window -t $SESSION:0
    tmux select-layout -t $SESSION:0 tiled
fi

tmux send -t $SESSION:0.0 "source ~/Diff-Swerve-ROS/scripts/set_client.sh wlp5s0 $HOST_IP" ENTER 
tmux send -t $SESSION:0.1 "source ~/Diff-Swerve-ROS/scripts/set_client.sh wlp5s0 $HOST_IP" ENTER 
tmux send -t $SESSION:0.2 "source ~/Diff-Swerve-ROS/scripts/set_client.sh wlp5s0 $HOST_IP" ENTER 
tmux send -t $SESSION:0.3 "source ~/Diff-Swerve-ROS/scripts/set_client.sh wlp5s0 $HOST_IP" ENTER 

tmux send -t $SESSION:0.0 "ssh -i $SSH_KEY_PATH tj2@$HOST_IP" ENTER 
tmux send -t $SESSION:0.0 "~/Diff-Swerve-ROS/scripts/tail-session.sh" ENTER 

tmux send -t $SESSION:0.1 "ssh -i $SSH_KEY_PATH tj2@$HOST_IP" ENTER 

tmux send -t $SESSION:0.2 "roslaunch tj2_debug_joystick tj2_debug_joystick.launch device:=$JOYSTICK_PATH topic_name:=joy_remote" ENTER 

tmux send -t $SESSION:0.3 "rviz" ENTER 

tmux a -t $SESSION
echo "Started local session"
