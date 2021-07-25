#!/bin/bash
tmux new -s record -d
tmux send -t record "roslaunch tj2_camera record_camera.launch" ENTER
tmux a -t record
