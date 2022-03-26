#!/usr/bin/python3
import csv
import math
import rospy
import tf2_ros
import tf2_geometry_msgs

import numpy as np

from scipy.interpolate import interp1d

from std_msgs.msg import Bool

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav_msgs.msg import Odometry

from tj2_tools.occupancy_grid import OccupancyGridManager

from tj2_waypoints.msg import WaypointArray

from tj2_networktables.msg import NTEntry

from tj2_tools.transforms import lookup_transform
from tj2_tools.robot_state import Pose2d, Velocity


class TJ2Turret(object):
    def __init__(self):
        self.node_name = "tj2_turret"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.map_frame = rospy.get_param("~map", "map")
        self.base_frame = rospy.get_param("~base_link", "base_link")
        self.turret_frame = rospy.get_param("~turret", "turret_tilt_link")
        self.target_waypoint = rospy.get_param("~target_waypoint", "center")

        self.x_std_threshold = rospy.get_param("~x_std_threshold", 1.0)
        self.y_std_threshold = rospy.get_param("~y_std_threshold", 1.0)
        self.theta_std_threshold = rospy.get_param("~theta_std_threshold", math.radians(45.0))

        self.enable_shot_correction = rospy.get_param("~enable_shot_correction", True)
        self.enable_shot_probability = rospy.get_param("~enable_shot_probability", True)

        self.time_of_flight_file_path = rospy.get_param("~time_of_flight_file_path", "./time_of_flight.txt")

        self.waypoints = {}
        
        self.hood_state = False  # False == down, True == up

        if self.enable_shot_correction:
            hood_up_table, hood_down_table = self.read_tof_file(self.time_of_flight_file_path)
            self.traj_interp_up = self.create_interp(hood_up_table)
            self.traj_interp_down = self.create_interp(hood_down_table)
        else:
            self.traj_interp_up = None
            self.traj_interp_down = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.nt_pub = rospy.Publisher("nt_passthrough", NTEntry, queue_size=10)
        self.turret_target_pub = rospy.Publisher("turret_target", PoseStamped, queue_size=10)

        self.waypoints_sub = rospy.Subscriber("waypoints", WaypointArray, self.waypoints_callback)
        self.amcl_pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.hood_sub = rospy.Subscriber("hood", Bool, self.hood_callback)

        if self.enable_shot_probability:
            self.ogm = OccupancyGridManager("map", subscribe_to_updates=True)
        else:
            self.ogm = None

        self.amcl_pose = None
        self.robot_velocity = Velocity()

        rospy.loginfo("%s init complete" % self.node_name)
    
    def read_tof_file(self, path):
        hood_up_table = []
        hood_down_table = []
        with open(path) as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                rospy.loginfo("Table row: %s" % str(row))
                parsed_row = list(map(float, row[1:]))
                if row[0] == "up":
                    hood_up_table.append(parsed_row)
                elif row[0] == "down":
                    hood_down_table.append(parsed_row)

        hood_up_table.sort(key=lambda x: x[0])
        hood_down_table.sort(key=lambda x: x[0])
        rospy.loginfo("Hood up table: %s" % str(hood_up_table))
        rospy.loginfo("Hood down table: %s" % str(hood_down_table))
        return np.array(hood_up_table), np.array(hood_down_table)
    
    def create_interp(self, table):
        if len(table) == 0:
            return None
        x_samples = table[:, 0]
        y_samples = table[:, 1]
        return interp1d(x_samples, y_samples, kind="linear")

    def waypoints_callback(self, msg):
        self.waypoints = {}
        for waypoint_msg in msg.waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_frame
            pose_stamped.pose = waypoint_msg.pose
            self.waypoints[waypoint_msg.name] = pose_stamped
    
    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg

    def odom_callback(self, msg):
        self.robot_velocity.x = msg.twist.twist.linear.x
        self.robot_velocity.y = msg.twist.twist.linear.y
        self.robot_velocity.theta = msg.twist.twist.angular.z
    
    def hood_callback(self, msg):
        self.hood_state = msg.data

    def run(self):
        rospy.sleep(1.0)  # wait for waypoints to populate
        clock = rospy.Rate(30.0)
        
        zero_pose_base = PoseStamped()
        zero_pose_base.header.frame_id = self.base_frame
        
        while not rospy.is_shutdown():
            if not self.is_global_pose_valid(self.amcl_pose):
                self.publish_turret(0.0, 0.0, 0.0)
                continue
            if self.target_waypoint not in self.waypoints:
                self.publish_turret(0.0, 0.0, 0.0)
                rospy.logwarn_throttle(1.0, "%s is not an available waypoint" % self.target_waypoint)
                continue
            target_pose_map = self.waypoints[self.target_waypoint]
            map_to_turret_tf = lookup_transform(self.tf_buffer, self.turret_frame, self.map_frame)
            if map_to_turret_tf is None:
                self.publish_turret(0.0, 0.0, 0.0)
                rospy.logwarn_throttle(1.0, "Unable to transfrom from %s -> %s" % (self.turret_frame, self.map_frame))
                continue
            target_pose_turret = tf2_geometry_msgs.do_transform_pose(target_pose_map, map_to_turret_tf)

            base_to_map_tf = lookup_transform(self.tf_buffer, self.map_frame, self.base_frame)
            if base_to_map_tf is None:
                self.publish_turret(0.0, 0.0, 0.0)
                rospy.logwarn_throttle(1.0, "Unable to transfrom from %s -> %s" % (self.map_frame, self.base_frame))
                continue
            robot_pose = tf2_geometry_msgs.do_transform_pose(zero_pose_base, base_to_map_tf)

            if self.enable_shot_probability:
                shot_probability = self.get_shot_probability(Pose2d.from_ros_pose(robot_pose.pose))
            else:
                shot_probability = 1.0

            target = Pose2d.from_ros_pose(target_pose_turret.pose)
            if self.enable_shot_correction:
                if self.hood_state:
                    traj_interp = self.traj_interp_up
                else:
                    traj_interp = self.traj_interp_down
                tof = self.get_tof(traj_interp, target.distance())
                if tof > 0.0:
                    target.x -= self.robot_velocity.x * tof
            target_heading = target.heading()
            target_heading = Pose2d.normalize_theta(target_heading + math.pi)
            self.publish_turret(target.distance(), target_heading, shot_probability)

            target_pose_stamped = PoseStamped()
            target_pose_stamped.header.frame_id = self.turret_frame
            target_pose_stamped.pose = target.to_ros_pose()
            self.turret_target_pub.publish(target_pose_stamped)

            clock.sleep()
    
    def get_tof(self, traj_interp, distance):
        # compute the time of flight of the ball to the target based on a lookup table
        try:
            if traj_interp is None:
                return 0.0
            else:
                return traj_interp(distance)
        except ValueError as e:
            rospy.logwarn_throttle(1.0, "Failed to compute TOF: %s" % str(e))
            return -1.0

    def get_shot_probability(self, pose2d: Pose2d):
        # see OccupancyGrid message docs
        # cost is -1 for unknown. Otherwise, 0..100 for probability
        # I remap this to 1.0..0.0
        if self.ogm is None:
            return 1.0

        cost = self.ogm.get_cost_from_world_x_y(pose2d.x, pose2d.y)
        if cost < 0.0:
            return 0.0
        else:
            return 1.0 - (cost / 100.0)
    
    def publish_turret(self, distance, heading, probability):
        self.nt_pub.publish(self.make_entry("turret/distance", distance))
        self.nt_pub.publish(self.make_entry("turret/heading", heading))
        self.nt_pub.publish(self.make_entry("turret/probability", probability))
        self.nt_pub.publish(self.make_entry("turret/update", rospy.Time.now().to_sec()))

    def is_global_pose_valid(self, amcl_pose):
        if amcl_pose is None or len(amcl_pose.pose.covariance) != 36:
            rospy.logwarn_throttle(1.0, "Invalid AMCL pose! %s" % str(amcl_pose))
            return False

        x_std = math.sqrt(amcl_pose.pose.covariance[0])
        y_std = math.sqrt(amcl_pose.pose.covariance[7])
        theta_std = math.sqrt(amcl_pose.pose.covariance[35])

        if (x_std < self.x_std_threshold and 
                y_std < self.y_std_threshold and 
                theta_std < self.theta_std_threshold):
            return True
        else:
            rospy.logwarn_throttle(1.0, "No valid turret target. stddev. x=%0.3f, y=%0.3f, theta=%0.3f" % (x_std, y_std, theta_std))
            return False
    
    def make_entry(self, path, value):
        entry = NTEntry()
        entry.path = path
        entry.value = value
        return entry


if __name__ == "__main__":
    node = TJ2Turret()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
