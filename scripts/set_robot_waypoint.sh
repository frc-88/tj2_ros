NAME=$1

rosservice call /tj2/tj2_waypoints/save_robot_pose "{name: $NAME}"
