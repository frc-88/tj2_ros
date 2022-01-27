rosservice call /tj2/set_launch "{name: move_base, mode: 1}"

SESSION=localize

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
fi

tmux send -t $SESSION "roslaunch tj2_bringup laser_slam.launch map_name:=$1 mode:=localize" ENTER
tmux a -t $SESSION
