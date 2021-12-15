#!/usr/bin/env bash

HOST_MACHINE=""

JUMP_THRESHOLD=60
TIMEOUT=${1:-900}

stop_time=$((SECONDS+TIMEOUT))
prev_time=$((SECONDS))
INTERFACE_NAME=eth0

while true; do
    HOST_MACHINE=`ifconfig $INTERFACE_NAME | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
    if [[ $HOST_MACHINE =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
        break
    fi
    sleep 0.5
    if (( $(echo "$SECONDS - $prev_time > $JUMP_THRESHOLD" |bc -l) )); then  # check if time jumped like when the jetson connects to the internet
        prev_time=$((SECONDS))
        stop_time=$((SECONDS+TIMEOUT))
        echo "Experienced a time jump. Resetting timeout"
    fi
    if [ $SECONDS -gt $stop_time ]; then
        echo "Failed to find host IP. Timed out after $stop_time seconds. Current time: $SECONDS"
        break
    fi
    prev_time=$((SECONDS))
done


export ROS_IP=${HOST_MACHINE}
export ROS_MASTER_URI=http://${HOST_MACHINE}:11311

echo ${ROS_IP}
echo ${ROS_MASTER_URI}

LOGPREFIX={$1:-log}
if [[ $TERM = "screen" ]] && [[ $(ps -p $PPID -o comm=) = tmux* ]]; then
    LOGPREFIX={$1:-log}
    echo "Enabling tmux logging for $LOGPREFIX"
    LOGDIR=$HOME/ros-logs
    mkdir $LOGDIR 2> /dev/null
    LOGNAME="$LOGPREFEX-$(date '+%Y-%m-%dT%H-%M-%S').log"
    script -f $LOGDIR/${LOGNAME}
    exit
fi

