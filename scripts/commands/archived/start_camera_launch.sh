#!/bin/bash
tmux new -s camera -d
tmux send -t camera "roslaunch tj2_camera tj2_camera.launch" ENTER
tmux a -t camera
