import os
from typing import Optional

import rospy
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped

from tj2_interfaces.msg import ZoneArray
from tj2_interfaces.msg import ZoneInfoArray
from tj2_interfaces.msg import NoGoZones

from tj2_tools.robot_state import Pose2d
from tj2_tools.zone import ZoneManager


class TJ2Zones:
    def __init__(self) -> None:
        self.zones_path = rospy.get_param("~zones_path", "")

        self.global_frame = rospy.get_param("~global_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")

        self.zones_pub = rospy.Publisher("zones", ZoneArray, queue_size=10)
        self.zones_info_pub = rospy.Publisher("zones_info", ZoneInfoArray, queue_size=10)
        self.zone_manager = ZoneManager.from_file(self.zones_path)
    
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.nogo_sub = rospy.Subscriber("nogo_zones", NoGoZones, self.nogo_callback, queue_size=5)
        self.nogo_zones = []

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
        self.nogo_zones = msg.nogo

    def run(self):
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            self.zones_pub.publish(self.zone_manager.to_msg())
            pose2d = self.get_robot_pose()
            if pose2d is None:
                info_msg = ZoneInfoArray()
                info_msg.is_valid = False
            else:
                info_msg = self.zone_manager.to_zone_info(pose2d)
            for info in info_msg.zones:
                info.is_nogo = info.zone.name in self.nogo_zones
            self.zones_pub.publish(info_msg)
            rate.sleep()
