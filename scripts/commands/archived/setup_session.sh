#!/bin/bash

SESSION=match

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
    tmux send -t $SESSION "roslaunch tj2_state_machine tj2_state_machine.launch" ENTER

    tmux split-window -h -t $SESSION
    tmux send -t $SESSION "rostopic hz /camera/depth/image_rect_raw" ENTER

    tmux split-window -v -t $SESSION
    tmux send -t $SESSION "roslaunch tj2_bar_pipeline tj2_bar_pipeline_py.launch" ENTER

else
    echo "Session is already setup"
fi
tmux a -t $SESSION
