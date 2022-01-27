BASE_DIR=$(realpath "$(dirname $0)")

SESSION=rtabmap
DB_PATH=$1

tmux has-session -t $SESSION > /dev/null

if [ $? != 0 ]; then
    tmux new -s $SESSION -d
fi

tmux send -t $SESSION "${BASE_DIR}/start_launch.sh rplidar
${BASE_DIR}/start_camera.sh" ENTER 

if [ -z ${DB_PATH} ]; then
    tmux send -t $SESSION "roslaunch tj2_rtabmap tj2_rtabmap.launch use_laser:=true" ENTER 
else
    tmux send -t $SESSION "roslaunch tj2_rtabmap tj2_rtabmap.launch use_laser:=true localization:=true database_path:=$DB_PATH" ENTER 
fi
echo "Started rtabmap"
