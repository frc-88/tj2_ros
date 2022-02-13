from smach import State


class NextWaypointState(State):
    """
    Increments to the next waypoint. If there are no more waypoints, exit
    """
    def __init__(self):
        super(NextWaypointState, self).__init__(
            outcomes=["success", "finished", "failure", "preempted"],
            input_keys=["waypoint_index_in"],
            output_keys=["waypoint_index_out"]
        )
