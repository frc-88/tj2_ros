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


class RecordTargetComparison(object):
    def __init__(self):
        self.name = "record_target_comparison"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.old_target_angle = 0.0
        self.old_target_dist = 0.0
        self.new_target_angle = 0.0
        self.new_target_dist = 0.0
        self.robot_velocity = Velocity()
        self.robot_pose = Pose2d()
        self.current_time = None

        self.file = open("new_vs_old_target_data.csv", 'w')
        self.writer = csv.DictWriter(self.file, fieldnames=("timestamp", "vx", "vt", "old_angle", "old_distance", "new_angle", "new_distance"))
        self.writer.writeheader()

        self.old_target_sub = rospy.Subscriber("/tj2/target_record", PoseStamped, self.old_target_callback)
        self.new_target_sub = rospy.Subscriber("/tj2/target", PoseStamped, self.new_target_callback)
        self.odom_sub = rospy.Subscriber("/tj2/odom", Odometry, self.odom_callback)
        self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_callback)

        rospy.loginfo("%s init complete" % self.name)
    
    def old_target_callback(self, msg):
        old_target = Pose2d.from_ros_pose(msg.pose)
        self.old_target_dist = old_target.distance(self.robot_pose)
        self.old_target_angle = old_target.heading(self.robot_pose)

    def new_target_callback(self, msg):
        new_target = Pose2d.from_ros_pose(msg.pose)
        self.new_target_dist = new_target.distance(self.robot_pose)
        self.new_target_angle = new_target.heading(self.robot_pose)

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
        self.robot_pose = Pose2d.from_ros_pose(global_pose.pose)
        row = {
            "timestamp": timestamp,
            "vx": self.robot_velocity.x,
            "vt": self.robot_velocity.theta,
            "old_angle": self.old_target_angle,
            "old_distance": self.old_target_dist,
            "new_angle": self.new_target_angle,
            "new_distance": self.new_target_dist,
        }
        self.writer.writerow(row)

    def run(self):
        clock = rospy.Rate(10.0)
        
        while not rospy.is_shutdown():
            self.record_turret()
            clock.sleep()

    def shutdown_hook(self):
        self.file.close()


if __name__ == "__main__":
    node = RecordTargetComparison()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.name)
