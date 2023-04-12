import rospy


class GridTracker:
    def __init__(self) -> None:
        self.node_name = "grid_tracker"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
