rostopic pub /tj2/follow_path/goal tj2_waypoints/FollowPathActionGoal "{goal: {waypoints: {waypoints: [{name: $1}]}, intermediate_tolerance: ${3:-0.25}}}" -1
