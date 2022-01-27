BASE_DIR=$(realpath "$(dirname $0)")

SESSION=ros
IP_INTERFACE=$1
HOST_IP=$2
SSH_KEY_PATH=$3
JOYSTICK_PATH=$4

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d

    tmux split-window -t $SESSION:0
    tmux split-window -t $SESSION:0
    tmux split-window -t $SESSION:0
    tmux select-layout -t $SESSION:0 tiled
fi

tmux send -t $SESSION:0.0 "ssh -i $SSH_KEY_PATH tj2@$HOST_IP" ENTER 
tmux send -t $SESSION:0.0 "~/tj2_ros/scripts/tail-session.sh" ENTER 

tmux send -t $SESSION:0.1 "ssh -i $SSH_KEY_PATH tj2@$HOST_IP" ENTER 

tmux send -t $SESSION:0.2 "source ~/tj2_ros/scripts/set_client.sh $IP_INTERFACE $HOST_IP" ENTER 
tmux send -t $SESSION:0.2 "roslaunch tj2_debug_joystick tj2_debug_joystick.launch device:=$JOYSTICK_PATH topic_name:=joy_remote" ENTER 

tmux send -t $SESSION:0.3 "source ~/tj2_ros/scripts/set_client.sh $IP_INTERFACE $HOST_IP" ENTER 
tmux send -t $SESSION:0.3 "rviz -d ~/tj2_ros/tj2_viz/rviz/standard.rviz" ENTER 

tmux a -t $SESSION
echo "Started local session"
