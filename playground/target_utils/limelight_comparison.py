#!/usr/bin/python3
import csv
import math

import rospy
import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from tj2_tools.robot_state import Pose2d, Velocity
from tj2_tools.transforms import lookup_transform


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
        rospy.on_shutdown(self.shutdown_hook)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.limelight_angle = 0.0
        self.limelight_dist = 0.0
        self.target_angle = 0.0
        self.target_dist = 0.0
        self.robot_velocity = Velocity()
        self.current_time = None

        self.file = open("target_data.csv", 'w')
        self.writer = csv.DictWriter(self.file, fieldnames=("timestamp", "x", "y", "theta", "target_angle", "target_distance", "limelight_angle", "limelight_distance"))
        self.writer.writeheader()

        self.limelight_angle_sub = rospy.Subscriber("/limelight/target_angle", Float64, self.limelight_angle_callback)
        self.limelight_dist_sub = rospy.Subscriber("/limelight/target_distance", Float64, self.limelight_dist_callback)
        self.target_angle_sub = rospy.Subscriber("/tj2/target_angle", Float64, self.target_angle_callback)
        self.target_dist_sub = rospy.Subscriber("/tj2/target_distance", Float64, self.target_dist_callback)
        self.odom_sub = rospy.Subscriber("/tj2/odom", Odometry, self.odom_callback)
        self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_callback)

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
    
    def clock_callback(self, msg):
        self.current_time = msg.clock
    
    def record_turret(self):
        if self.current_time is None:
            return
        timestamp = self.current_time.to_sec()
        global_tf = lookup_transform(self.tf_buffer, "map", "base_link")
        if global_tf is None:
            return

        zero_pose_base = PoseStamped()
        zero_pose_base.header.frame_id = "base_link"
        zero_pose_base.pose.orientation.w = 1.0

        global_pose = tf2_geometry_msgs.do_transform_pose(zero_pose_base, global_tf)
        global_pose_2d = Pose2d.from_ros_pose(global_pose.pose)
        row = {
            "timestamp": timestamp,
            "x": global_pose_2d.x,
            "y": global_pose_2d.y,
            "theta": global_pose_2d.theta,
            "target_angle": self.target_angle,
            "target_distance": self.target_dist,
            "limelight_angle": self.limelight_angle,
            "limelight_distance": self.limelight_dist,
        }
        self.writer.writerow(row)

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
                self.record_turret()

            if self.robot_velocity.distance() < 0.1:
                limelight_angle_deg = math.degrees(self.limelight_angle)
                target_angle_deg = math.degrees(self.target_angle)
                delta = limelight_angle_deg - target_angle_deg
                # print("Limelight (%0.4f) - Waypoint (%0.4f) = %0.4f deg" % (limelight_angle_deg, target_angle_deg, delta))
                print(delta)
            clock.sleep()

    def shutdown_hook(self):
        self.file.close()


if __name__ == "__main__":
    node = LimelightComparison()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.name)
