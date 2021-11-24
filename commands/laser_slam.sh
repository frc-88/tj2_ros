BASE_DIR=$(realpath "$(dirname $0)")

SESSION=slam
MAP_NAME=$1

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
fi

tmux send -t $SESSION "${BASE_DIR}/start_launch.sh rplidar" ENTER 
if [ -z ${MAP_NAME} ]; then
    tmux send -t $SESSION "roslaunch tj2_laser_slam tj2_laser_slam.launch" ENTER 
else
    tmux send -t $SESSION "roslaunch tj2_laser_slam tj2_laser_slam.launch map_name:=$MAP_NAME mode:=localize" ENTER 
fi
echo "Started laser slam"
