import rospy
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

    def execute(self, userdata):
        userdata.waypoint_index_out = userdata.waypoint_index_in + 1

        num_waypoints = len(userdata.waypoints_plan)

        if userdata.waypoint_index_out >= num_waypoints:
            return "finished"
        else:
            rospy.loginfo("Selecting next waypoint %s -> %s" % (userdata.waypoint_index_in, userdata.waypoint_index_out))
            return "success"
