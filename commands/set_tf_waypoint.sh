NAME=$1
FRAME=$2

rosservice call /tj2/tj2_waypoints/save_tf "{name: $NAME, frame: $FRAME}"
