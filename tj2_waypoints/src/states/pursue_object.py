
from smach import State


class PursueObjectState(State):
    """
    Runs the pursuit planner searching for detections with the supplied waypoint name
    """
    def __init__(self):
        super(PursueObjectState, self).__init__(
            outcomes=["success", "preempted", "failure"],
            input_keys=["waypoints_plan", "waypoint_index", "state_machine"],
            output_keys=["waypoints_plan", "waypoint_index", "state_machine"]
        )
