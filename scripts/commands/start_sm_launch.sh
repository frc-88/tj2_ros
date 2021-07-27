#!/bin/bash
tmux new -s sm -d
tmux send -t sm "roslaunch tj2_state_machine tj2_state_machine.launch" ENTER
tmux a -t sm
