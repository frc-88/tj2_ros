BASE_DIR=$(realpath "$(dirname $0)")

source ${BASE_DIR}/startup.sh
roslaunch tj2_match_watcher play_match_bag.launch bag_name:=$1 start_time:=${2:-0}
