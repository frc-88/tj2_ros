#!/usr/bin/python3
import os
import cv2
import rospy
import rospkg

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped

from nav_msgs.srv import GetMap

from tj2_waypoints.msg import WaypointArray

from tj2_turret.srv import SetProbability, SetProbabilityResponse
from tj2_turret.srv import SetProbabilityRing, SetProbabilityRingResponse

from tj2_tools.transforms import lookup_transform

from tj2_tools.occupancy_grid import OccupancyGridManager
from tj2_tools.launch_manager import LaunchManager

from tj2_tools.robot_state import Pose2d


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
        self.robot_radius = rospy.get_param("~robot_radius", 0.1)

        self.map_frame = rospy.get_param("~map", "map")
        self.base_frame = rospy.get_param("~base_link", "base_link")

        self.map_name = rospy.get_param("~map_name", "turret_map.yaml")

        self.waypoints = {}

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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.rospack = rospkg.RosPack()
        self.package_dir = self.rospack.get_path("tj2_turret")
        self.default_launches_dir = os.path.join(self.package_dir, "launch/sublaunch")
        self.updated_map_launch_path = os.path.join(self.default_launches_dir, "/updated_map.launch")
        self.updated_map_launcher = LaunchManager(self.updated_map_launch_path)

        self.map_saver_launcher = LaunchManager(self.default_launches_dir + "/map_saver.launch")
        self.map_server_launcher = LaunchManager(self.default_launches_dir + "/map_server.launch")

        if self.map_name.startswith("/"):
            self.map_path = self.map_name
        else:
            self.map_path = os.path.join(self.package_dir, "maps/" + self.map_name)
        self.map_dir = os.path.dirname(self.map_path)
        if not os.path.isdir(self.map_dir):
            os.makedirs(self.map_dir)
        
        self.map_saver_wait_time = 14.0

        self.save_map()

        self.set_probability_srv = self.make_service("probability_map/set_probability", SetProbability, self.set_probability)
        self.set_probability_ring_srv = self.make_service("probability_map/set_probability_ring", SetProbabilityRing, self.set_probability_ring)

        self.waypoints_sub = rospy.Subscriber("waypoints", WaypointArray, self.waypoints_callback)

        rospy.loginfo("%s is ready" % self.name)
    
    def set_probability(self, req):
        robot_x, robot_y = self.get_robot_map_xy()
        radius = int(self.robot_radius * self.ogm.resolution)
        color = int(min(1.0, max(0.0, req.probability)) * 255)
        cv2.circle(self.ogm.grid_data, (robot_x, robot_y), radius, (color,), -1)
        self.save_map()
        return SetProbabilityResponse(True)

    def set_probability_ring(self, req):
        reference_pose = self.waypoints[req.waypoint_name]
        ref_pose = Pose2d(*self.ogm.get_costmap_x_y(
            reference_pose.pose.position.x,
            reference_pose.pose.position.y,
        ))
        robot_pose = Pose2d(*self.get_robot_map_xy())
        color = int(min(1.0, max(0.0, req.probability)) * 255)
        radius = int(ref_pose.distance(robot_pose) * self.ogm.resolution)
        cv2.circle(self.ogm.grid_data, (ref_pose.x, ref_pose.y), radius, (color,), -1)
        self.save_map()
        return SetProbabilityRingResponse(True)
    
    def get_robot_map_xy(self):
        pose = self.get_robot_pose()
        x = pose.pose.position.x
        y = pose.pose.position.y
        map_x, map_y = self.ogm.get_costmap_x_y(x, y)
        return map_x, map_y

    def get_robot_pose(self):
        base_to_map_tf = lookup_transform(self.tf_buffer, self.map_frame, self.base_frame)
        if base_to_map_tf is None:
            rospy.logwarn_throttle(1.0, "Unable to transfrom from %s -> %s" % (self.map_frame, self.base_frame))
        zero_pose_base = PoseStamped()
        zero_pose_base.header.frame_id = self.base_frame

        return tf2_geometry_msgs.do_transform_pose(zero_pose_base, base_to_map_tf)

    def probability_to_cost(self, probability):
        probability = min(1.0, max(0.0, probability))
        probability = 1.0 - probability
        cost = int(100 * probability)
        return cost

    def cost_to_probability(self, cost):
        cost = min(100, max(0, cost))
        cost = 100 - cost
        probability = cost / 100
        return probability

    def save_map(self):
        self.map_server_launcher.stop()
        self.map_saver_launcher.set_args(map_path=self.map_path)
        self.map_saver_launcher.start()
        self.map_saver_launcher.join(timeout=self.map_saver_wait_time)
        self.map_server_launcher.start()

    def waypoints_callback(self, msg):
        self.waypoints = {}
        for waypoint_msg in msg.waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_frame
            pose_stamped.pose = waypoint_msg.pose
            self.waypoints[waypoint_msg.name] = pose_stamped
    
    def make_service(self, name, srv_type, callback):
        rospy.loginfo("Setting up service %s" % name)
        srv_obj = rospy.Service(name, srv_type, callback)
        rospy.loginfo("%s service is ready" % name)
        return srv_obj

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

    def run(self):
        rospy.spin()
    

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
