from smach import State


class CheckWaypointState(State):
    """
    Checks the current waypoint to see if it's a spot on the map or the name of an object to pursue
    """
    def __init__(self):
        super(CheckWaypointState, self).__init__(
            outcomes=["waypoint", "object", "finished", "failure", "preempted"],
            input_keys=["waypoints_plan", "waypoint_index", "state_machine"],
            output_keys=["waypoints_plan", "waypoint_index", "state_machine"]
        )
