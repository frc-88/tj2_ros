#!/usr/bin/env bash
echo "Stopping roslaunch!"

SESSION=roslaunch

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    echo "roslaunch is already stopped."
    exit
fi

tmux send-keys -t $SESSION C-c

while true
do
    tmux has-session -t $SESSION > /dev/null

    if [ $? != 0 ]; then
        echo "roslaunch exited"
        exit
    fi
    tmux send-keys -t $SESSION C-d
    sleep 1
done
