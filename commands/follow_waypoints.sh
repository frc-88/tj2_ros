rostopic pub /tj2/follow_path/goal tj2_waypoints/FollowPathActionGoal "{goal: {waypoints: [$1], is_continuous: ${2:-true}, intermediate_tolerance: ${3:-0.25}}}" -1
