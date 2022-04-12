#!/usr/bin/python3
import os
import csv
import math

from numpy import concatenate
import rospy
import tf2_ros
import tf2_geometry_msgs

import numpy as np

from scipy.interpolate import interp1d

from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

from tj2_tools.occupancy_grid import OccupancyGridManager

from tj2_waypoints.msg import WaypointArray

from tj2_networktables.msg import NTEntry

from tj2_target.srv import RecordValue, RecordValueResponse
from std_srvs.srv import Trigger, TriggerResponse

from tj2_tools.transforms import lookup_transform
from tj2_tools.robot_state import Pose2d, Velocity


def meters_to_in(meters):
    return meters * 39.37


class LimelightComparison(object):
    def __init__(self):
        self.name = "limelight_comparison"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        self.limelight_angle = 0.0
        self.limelight_dist = 0.0
        self.target_angle = 0.0
        self.target_dist = 0.0
        self.robot_velocity = Velocity()

        self.limelight_angle_sub = rospy.Subscriber("/limelight/target_angle", Float64, self.limelight_angle_callback)
        self.limelight_dist_sub = rospy.Subscriber("/limelight/target_distance", Float64, self.limelight_dist_callback)
        self.target_angle_sub = rospy.Subscriber("/tj2/target_angle", Float64, self.target_angle_callback)
        self.target_dist_sub = rospy.Subscriber("/tj2/target_distance", Float64, self.target_dist_callback)
        self.odom_sub = rospy.Subscriber("/tj2/odom", Odometry, self.odom_callback)
        self.limelight_pub = rospy.Publisher("debug_limelight_target", PoseStamped, queue_size=10)
        self.target_pub = rospy.Publisher("debug_target", PoseStamped, queue_size=10)

        rospy.loginfo("%s init complete" % self.name)
    
    def limelight_angle_callback(self, msg):
        self.limelight_angle = msg.data + math.pi

    def limelight_dist_callback(self, msg):
        self.limelight_dist = msg.data

    def target_angle_callback(self, msg):
        self.target_angle = msg.data + math.pi

    def target_dist_callback(self, msg):
        self.target_dist = msg.data

    def odom_callback(self, msg):
        self.robot_velocity.x = msg.twist.twist.linear.x
        self.robot_velocity.y = msg.twist.twist.linear.y
        self.robot_velocity.theta = msg.twist.twist.angular.z
    
    def run(self):
        clock = rospy.Rate(10.0)
        
        while not rospy.is_shutdown():
            if not math.isnan(self.limelight_angle):
                limelight_x = self.limelight_dist * math.cos(self.limelight_angle)
                limelight_y = self.limelight_dist * math.sin(self.limelight_angle)
                limelight_pose2d = Pose2d(limelight_x, limelight_y)
                limelight_pose = PoseStamped()
                limelight_pose.header.frame_id = "turret_tilt_link"
                limelight_pose.pose = limelight_pose2d.to_ros_pose()
                self.limelight_pub.publish(limelight_pose)

            if not math.isnan(self.target_angle):
                target_x = self.target_dist * math.cos(self.target_angle)
                target_y = self.target_dist * math.sin(self.target_angle)
                target_pose2d = Pose2d(target_x, target_y)
                target_pose = PoseStamped()
                target_pose.header.frame_id = "turret_tilt_link"
                target_pose.pose = target_pose2d.to_ros_pose()
                self.target_pub.publish(target_pose)

            if self.robot_velocity.distance() < 0.1:
                limelight_angle_deg = math.degrees(self.limelight_angle)
                target_angle_deg = math.degrees(self.target_angle)
                delta = limelight_angle_deg - target_angle_deg
                # print("Limelight (%0.4f) - Waypoint (%0.4f) = %0.4f deg" % (limelight_angle_deg, target_angle_deg, delta))
                print(delta)
            clock.sleep()


if __name__ == "__main__":
    node = LimelightComparison()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.name)
