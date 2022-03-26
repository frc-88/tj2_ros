import os
import rospy
import rospkg

from nav_msgs.srv import GetMap

from tj2_tools.occupancy_grid import OccupancyGridManager
from tj2_tools.launch_manager import LaunchManager


class ProbabilityMapTool:
    def __init__(self):
        self.name = "probability_map_tool"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.turret_map_service_name = rospy.get_param("~turret_map_service", "/turret/static_map")
        self.main_map_service_name = rospy.get_param("~main_map_service", "/static_map")

        try:
            self.get_turret_map = self.make_service_client(self.turret_map_service_name, GetMap, timeout=2.0)
        except rospy.ROSException:
            rospy.loginfo("Turret map does not exist, creating a new map from the main map")
            self.get_turret_map = lambda: None
        self.map_msg = self.get_turret_map()

        if self.map_msg is not None:
            self.ogm = OccupancyGridManager(self.map_msg.map)
        else:
            self.get_main_map = self.make_service_client(self.main_map_service_name, GetMap, timeout=2.0)
            self.map_msg = self.get_main_map()
            self.ogm = OccupancyGridManager(self.map_msg.map)
            self.ogm.grid_data[:] = -1  # by default set all grid data to unknown for new map

        self.rospack = rospkg.RosPack()
        self.package_dir = self.rospack.get_path(self.name)
        self.default_launches_dir = os.path.join(self.package_dir, "/launch/sublaunch")
        self.updated_map_launch_path = os.path.join(self.default_launches_dir, "/updated_map.launch")
        self.updated_map_launcher = LaunchManager(self.updated_map_launch_path)

        self.map_saver_launch_path = self.default_launches_dir + "/map_saver.launch"
        self.map_saver_launcher = LaunchManager(self.map_saver_launch_path)

        self.temp_map_path = os.path.join(self.package_dir, "maps/updated_map.yaml")
        self.map_dir = os.path.dirname(self.temp_map_path)
        if not os.path.isdir(self.map_dir):
            os.makedirs(self.map_dir)
        
        self.map_saver_wait_time = 14.0

        rospy.loginfo("%s is ready" % self.name)

    def save_temp_map(self):
        self.map_saver_launcher.set_args(map_path=self.temp_map_path)
        self.map_saver_launcher.start()
        self.map_saver_launcher.join(timeout=self.map_saver_wait_time)

    def make_service_client(self, name, srv_type, timeout=None):
        """
        Create a ros service client. Optionally wait with or without a timeout for the server to connect
        """
        self.__dict__[name + "_service_name"] = name
        rospy.loginfo("Connecting to %s service" % name)
        srv_obj = rospy.ServiceProxy(name, srv_type)

        if timeout is not None:
            rospy.loginfo("Waiting for service %s" % name)
            rospy.wait_for_service(name, timeout=timeout)
            rospy.loginfo("%s service is ready" % name)
        return srv_obj
    

def main():
    node = ProbabilityMapTool()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)

if __name__ == "__main__":
    main()
