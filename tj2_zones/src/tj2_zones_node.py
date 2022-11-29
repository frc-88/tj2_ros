#!/usr/bin/env python3
from typing import Optional

import rospy
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped

from tj2_interfaces.msg import Zone
from tj2_interfaces.msg import ZoneArray
from tj2_interfaces.msg import ZoneInfoArray
from tj2_interfaces.msg import NoGoZones

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

from tj2_tools.robot_state import Pose2d
from tj2_tools.zone import ZoneManager
from tj2_tools.occupancy_grid import OccupancyGridManager


class TJ2Zones:
    def __init__(self) -> None:
        rospy.init_node(
            "tj2_zones",
            # disable_signals=True,
            # log_level=rospy.DEBUG
        )
        self.zones_path = rospy.get_param("~zones_path", "")

        self.global_frame = rospy.get_param("~global_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")

        self.occupied_value = rospy.get_param("~occupied", 100)
        self.free_value = rospy.get_param("~free", 0)
        self.nogo_zones_param = rospy.get_param("~nogos", None)
        if self.nogo_zones_param is None:
            self.nogo_zones_param = []

        self.zones_pub = rospy.Publisher("zones", ZoneArray, queue_size=10)
        self.zones_info_pub = rospy.Publisher("zones_info", ZoneInfoArray, queue_size=10)
        self.zone_map_pub = rospy.Publisher("zone_map", OccupancyGrid, queue_size=10)
        self.zone_map_update_pub = rospy.Publisher("zone_map_updates", OccupancyGridUpdate, queue_size=10)
        self.zone_manager = ZoneManager.from_file(self.zones_path)
        self.zone_manager.update_nogos([name.data for name in self.nogo_zones_param])
    
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.ogm = OccupancyGridManager()
        
        self.zones_changed = True

        self.update_zones_sub = rospy.Subscriber("update_zones", ZoneArray, self.update_zones_callback, queue_size=5)
        self.add_zone_sub = rospy.Subscriber("add_zone", Zone, self.add_zone_callback, queue_size=5)
        self.nogo_sub = rospy.Subscriber("nogo_zones", NoGoZones, self.nogo_callback, queue_size=5)
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=5)

    def get_robot_pose(self) -> Optional[Pose2d]:
        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return None
        robot_pose = PoseStamped()
        robot_pose.pose.orientation.w = 1.0
        map_pose = tf2_geometry_msgs.do_transform_pose(robot_pose, transform)
        return Pose2d.from_ros_pose(map_pose.pose)

    def nogo_callback(self, msg):
        nogo_zones = [name.data for name in msg.nogo]
        if self.zone_manager.get_nogos() != nogo_zones:
            rospy.loginfo(f"Setting no-go zones: {nogo_zones}")
            self.zone_manager.update_nogos(nogo_zones)
            self.zones_changed = True

    def map_callback(self, msg):
        rospy.loginfo(f"Setting base map: {msg.header}. {msg.info}.")
        self.ogm = OccupancyGridManager.from_msg(msg)

    def update_zones_callback(self, msg: ZoneArray):
        self.zone_manager.update_zones(msg)
        self.zones_changed = True

    def add_zone_callback(self, msg: Zone):
        self.zone_manager.add_zone(msg)
        self.zones_changed = True

    def publish_as_map(self):
        rospy.loginfo("Publishing new zones map")
        if not self.ogm.is_set():
            rospy.loginfo("Occupancy grid manager isn't set!")
            return False
        zone_ogm = OccupancyGridManager.from_ogm(self.ogm)
        zone_ogm.set_grid_data(self.zone_manager.to_grid_data(self.ogm, self.free_value, self.occupied_value))
        map_msg = zone_ogm.to_msg()
        self.zone_map_pub.publish(map_msg)
        
        update_msg = OccupancyGridUpdate()
        update_msg.x = 0
        update_msg.y = 0
        update_msg.width = map_msg.info.width
        update_msg.height = map_msg.info.height
        update_msg.data = map_msg.data
        self.zone_map_update_pub.publish(update_msg)
        
        return True

    def run(self):
        rate = rospy.Rate(5.0)
        num_map_subs = 0
        while not rospy.is_shutdown():
            current_map_subs = \
                self.map_sub.get_num_connections() + \
                self.zone_map_pub.get_num_connections() + \
                self.zone_map_update_pub.get_num_connections()
            
            if self.zones_changed or (current_map_subs != num_map_subs and num_map_subs > 0):
                num_map_subs = current_map_subs
                if self.publish_as_map():
                    self.zones_changed = False
            self.zones_pub.publish(self.zone_manager.to_msg())
            pose2d = self.get_robot_pose()
            if pose2d is None:
                info_msg = ZoneInfoArray()
                info_msg.is_valid = False
            else:
                info_msg = self.zone_manager.to_zone_info(pose2d)
            self.zones_info_pub.publish(info_msg)
            rate.sleep()


if __name__ == '__main__':
    node = TJ2Zones()
    node.run()
