import rospy
from grid_zone_manager import GridZoneManager


class GridTracker:
    def __init__(self) -> None:
        self.node_name = "grid_tracker"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        grid_zones_path = rospy.get_param("~grid_zones_path", "grid_zones.csv")
        self.grid_zones = GridZoneManager.from_file(grid_zones_path)
