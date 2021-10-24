#!/usr/bin/env bash

HOST_MACHINE=""

JUMP_THRESHOLD=60
TIMEOUT=${1:-900}

stop_time=$((SECONDS+TIMEOUT))
prev_time=$((SECONDS))

while true; do
    HOST_MACHINE=`ifconfig eth0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
    if [[ $HOST_MACHINE =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
        break
    fi
    # HOST_MACHINE=`ifconfig wlan0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'`
    # if [ ! -z ${HOST_MACHINE} ]; then
    #     break
    # fi
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
# export ROS_MASTER_URI=http://${HOST_MACHINE_NAME}:11311

echo ${ROS_IP}
echo ${ROS_MASTER_URI}
