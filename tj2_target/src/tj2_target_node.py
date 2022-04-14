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

from tj2_target.msg import RevColorSensor
from tj2_target.msg import TargetConfig

from tj2_target.srv import RecordValue, RecordValueResponse
from std_srvs.srv import Trigger, TriggerResponse

from tj2_tools.transforms import lookup_transform
from tj2_tools.robot_state import Pose2d, Velocity


def meters_to_in(meters):
    return meters * 39.37


class TJ2Target(object):
    def __init__(self):
        self.node_name = "tj2_target"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.map_frame = rospy.get_param("~map", "map")
        self.odom_frame = rospy.get_param("~odom", "odom")
        self.base_frame = rospy.get_param("~base_link", "base_link")
        self.target_base_frame = rospy.get_param("~target_base_frame", "base_link")
        self.target_waypoint = rospy.get_param("~target_waypoint", "center")
        self.marauding_target_prefix = rospy.get_param("~marauding_target_prefix", "<team>_maraude")

        self.x_cov_threshold = rospy.get_param("~x_cov_threshold", 1.0)
        self.y_cov_threshold = rospy.get_param("~y_cov_threshold", 1.0)
        self.theta_cov_threshold = math.radians(rospy.get_param("~theta_cov_threshold_deg", 45.0))

        self.enable_shot_correction = rospy.get_param("~enable_shot_correction", True)
        self.enable_shot_probability = rospy.get_param("~enable_shot_probability", False)
        self.enable_limelight_fine_tuning = rospy.get_param("~enable_limelight_fine_tuning", False)
        self.enable_marauding = rospy.get_param("~enable_marauding", False)
        self.enable_reset_to_limelight = rospy.get_param("~enable_reset_to_limelight", False)

        # a constant to fix weird unknown target issues
        self.target_cosmic_ray_compensation = math.radians(rospy.get_param("~target_cosmic_ray_compensation_degrees", 0.0))

        self.color_message_timeout = rospy.Duration(rospy.get_param("~color_message_timeout", 0.5))
        self.cargo_egress_timeout = rospy.Duration(rospy.get_param("~cargo_egress_timeout", 0.5))
        self.cargo_egress_timer = rospy.Time.now()

        self.reset_x_cov = rospy.get_param("~reset_x_cov", 1.0)
        self.reset_y_cov = rospy.get_param("~reset_y_cov", 1.0)
        self.reset_theta_cov = math.radians(rospy.get_param("~reset_theta_cov_deg", 45.0))
        self.stale_limelight_timeout = rospy.Duration(rospy.get_param("~stale_limelight_timeout", 0.5))
        self.limelight_waypoint_agreement_threshold = rospy.get_param("~limelight_waypoint_agreement_threshold", 1.0)

        self.time_of_flight_file_path = rospy.get_param("~time_of_flight_file_path", "./time_of_flight.csv")
        self.recorded_data_file_path = rospy.get_param("~recorded_data_file_path", "./recorded_data.csv")
        self.probability_hood_up_map_path = rospy.get_param("~probability_hood_up_map_path", "./probability.yaml")
        self.probability_hood_down_map_path = rospy.get_param("~probability_hood_down_map_path", "./probability.yaml")

        self.waypoints = {}
        self.waypoints_pose2d = {}
        self.target_heading = 0.0
        self.target_distance = 0.0
        self.robot_pose = PoseStamped()
        self.hood_state = False  # False == down, True == up
        self.limelight_target_pose = PoseStamped()
        self.map_to_odom_tf_at_reset = None

        self.amcl_pose = None
        self.robot_velocity = Velocity()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        if self.enable_shot_correction:
            hood_up_table, hood_down_table = self.read_tof_file(self.time_of_flight_file_path)
            self.traj_interp_up = self.create_interp(hood_up_table)
            self.traj_interp_down = self.create_interp(hood_down_table)
        else:
            self.traj_interp_up = None
            self.traj_interp_down = None
        
        self.loaded_object_color = ""
        self.buffered_object_color = ""  # cargo takes time to leave the chamber. Hold the last object color here
        self.color_timestamp = rospy.Time(0)
        self.team_timestamp = rospy.Time(0)

        self.team_color = ""

        self.nt_pub = rospy.Publisher("nt_passthrough", NTEntry, queue_size=10)
        self.target_pub = rospy.Publisher("target", PoseStamped, queue_size=10)
        self.probability_hood_up_pub = rospy.Publisher("shot_probability_hood_up_map", OccupancyGrid, queue_size=10)
        self.probability_hood_down_pub = rospy.Publisher("shot_probability_hood_down_map", OccupancyGrid, queue_size=10)
        self.target_angle_pub = rospy.Publisher("target_angle", Float64, queue_size=15)
        self.target_distance_pub = rospy.Publisher("target_distance", Float64, queue_size=15)
        self.target_probability_pub = rospy.Publisher("target_probability", Float64, queue_size=15)
        self.initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=15)

        self.waypoints_sub = rospy.Subscriber("waypoints", WaypointArray, self.waypoints_callback)
        self.amcl_pose_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.hood_sub = rospy.Subscriber("hood", Bool, self.hood_callback)
        self.limelight_sub = rospy.Subscriber("/limelight/target", PoseStamped, self.limelight_callback)
        self.color_sensor_sub = rospy.Subscriber("/color_sensor/sensor", RevColorSensor, self.color_sensor_callback)
        self.team_color_sub = rospy.Subscriber("team_color", String, self.team_color_callback)
        self.reset_to_limelight_sub = rospy.Subscriber("reset_to_limelight", Float64, self.reset_to_limelight_callback)
        self.reset_pose_sub = rospy.Subscriber("reset_pose", PoseWithCovarianceStamped, self.reset_pose_callback)
        self.target_config_sub = rospy.Subscriber("target_config", TargetConfig, self.target_config_callback)

        if self.enable_shot_probability:
            self.ogm_up = OccupancyGridManager.from_cost_file(self.probability_hood_up_map_path)
            self.ogm_down = OccupancyGridManager.from_cost_file(self.probability_hood_down_map_path)
            self.map_pub_timer = rospy.Timer(rospy.Duration(1.0), self.map_publish_callback)
        else:
            self.ogm_up = None
            self.ogm_down = None
            self.map_pub_timer = None
        
        self.record_tof_srv = self.make_service("record_tof", RecordValue, self.record_tof)
        self.record_probability_srv = self.make_service("record_probability", RecordValue, self.record_probability)
        self.record_wheel_srv = self.make_service("record_flywheel", RecordValue, self.record_flywheel)
        self.convert_to_tof_srv = self.make_service("convert_to_tof_file", Trigger, self.convert_to_tof_file)
        self.convert_to_flywheel_srv = self.make_service("convert_to_flywheel", Trigger, self.convert_to_flywheel_file)

        recorded_dir = os.path.dirname(self.recorded_data_file_path)
        if not os.path.isdir(recorded_dir):
            os.makedirs(recorded_dir)
        if not os.path.isfile(self.recorded_data_file_path):
            self.create_data_file(self.recorded_data_file_path)


        rospy.loginfo("%s init complete" % self.node_name)
    
    def make_service(self, name, srv_type, callback):
        rospy.loginfo("Setting up service %s" % name)
        srv_obj = rospy.Service(name, srv_type, callback)
        rospy.loginfo("%s service is ready" % name)
        return srv_obj

    def record_tof(self, req):
        tof = req.value
        robot_pose = Pose2d.from_ros_pose(self.robot_pose.pose)
        self.record_row("tof", tof, robot_pose)
        return RecordValueResponse(True)

    def record_probability(self, req):
        probability = req.value
        robot_pose = Pose2d.from_ros_pose(self.robot_pose.pose)
        self.record_row("prob", probability, robot_pose)
        return RecordValueResponse(True)
    
    def record_flywheel(self, req):
        speed = req.value
        robot_pose = Pose2d.from_ros_pose(self.robot_pose.pose)
        self.record_row("flywheel", speed, robot_pose)
        return RecordValueResponse(True)
    
    def convert_to_tof_file(self, req):
        data = []
        with open(self.recorded_data_file_path) as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                row_type = row[header.index("type")]
                if row_type != "tof":
                    continue
                hood_state_str = row[header.index("hood")]
                distance = float(row[header.index("distance")])
                tof = float(row[header.index("value")])
                data.append([hood_state_str, distance, tof])
        new_path = os.path.splitext(self.time_of_flight_file_path)[0]
        new_path += "-new.csv"
        rospy.loginfo("Writing table to %s" % str(new_path))
        with open(new_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["hood", "distance", "time"])
            for row in data:
                writer.writerow(row)
        return TriggerResponse()
    
    def convert_to_flywheel_file(self, req):
        with open(self.recorded_data_file_path) as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                row_type = row[header.index("type")]
                if row_type != "flywheel":
                    continue
                hood_state_str = row[header.index("hood")]
                distance = float(row[header.index("distance")])
                speed = float(row[header.index("value")])
                print("%s\t%s\t%s" % (hood_state_str, distance, speed))
        
        return TriggerResponse()
    
    def record_row(self, row_type, value, robot_pose):
        row = {
            "type": row_type,
            "hood": "up" if self.hood_state else "down",
            "distance": self.target_distance,
            "heading": self.target_heading,
            "value": value,
            "x": robot_pose.x,
            "y": robot_pose.y,
            "theta": robot_pose.theta
        }
        print("Writing: %s" % str(row))
        with open(self.recorded_data_file_path, 'a', newline='') as file:
            writer = self.get_data_writer(file)
            writer.writerow(row)
    
    def create_data_file(self, path):
        with open(path, 'w', newline='') as file:
            writer = self.get_data_writer(file)
            writer.writeheader()
        
    def get_data_writer(self, file):
        return csv.DictWriter(file, fieldnames=("type", "hood", "distance", "heading", "value", "x", "y", "theta"))
    
    def read_tof_file(self, path):
        hood_up_table = []
        hood_down_table = []
        rospy.loginfo("Loading tof table from %s" % path)
        with open(path) as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                rospy.logdebug("Table row: %s" % str(row))
                parsed_row = list(map(float, row[1:]))
                if row[0] == "up":
                    hood_up_table.append(parsed_row)
                elif row[0] == "down":
                    hood_down_table.append(parsed_row)

        hood_up_table.sort(key=lambda x: x[0])
        hood_down_table.sort(key=lambda x: x[0])
        rospy.logdebug("Hood up table: %s" % str(hood_up_table))
        rospy.logdebug("Hood down table: %s" % str(hood_down_table))
        return np.array(hood_up_table), np.array(hood_down_table)
    
    def create_interp(self, table):
        if len(table) == 0:
            return None
        x_samples = table[:, 0]
        y_samples = table[:, 1]
        return interp1d(x_samples, y_samples, kind="linear")

    def map_publish_callback(self, timer):
        self.probability_hood_up_pub.publish(self.ogm_up.to_msg())
        self.probability_hood_down_pub.publish(self.ogm_down.to_msg())

    def waypoints_callback(self, msg):
        self.waypoints = {}
        self.waypoints_pose2d = {}
        for waypoint_msg in msg.waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_frame
            pose_stamped.pose = waypoint_msg.pose
            self.waypoints[waypoint_msg.name] = pose_stamped
            self.waypoints_pose2d[waypoint_msg.name] = Pose2d.from_ros_pose(pose_stamped.pose)
    
    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg

    def odom_callback(self, msg):
        self.robot_velocity.x = msg.twist.twist.linear.x
        self.robot_velocity.y = msg.twist.twist.linear.y
        self.robot_velocity.theta = msg.twist.twist.angular.z
    
    def hood_callback(self, msg):
        self.hood_state = msg.data
    
    def limelight_callback(self, msg):
        self.limelight_target_pose = msg

    def color_sensor_callback(self, msg):
        self.color_timestamp = rospy.Time.now()
        self.loaded_object_color = msg.match

    def team_color_callback(self, msg):
        self.team_timestamp = rospy.Time.now()
        self.team_color = msg.data
    
    def reset_to_limelight_callback(self, msg):
        if self.enable_reset_to_limelight:
            return
        turret_to_odom_tf = lookup_transform(self.tf_buffer, self.odom_frame, self.limelight_target_pose.header.frame_id)
        if turret_to_odom_tf is None:
            return
        if self.map_to_odom_tf_at_reset is None:
            return
        dt = rospy.Time.now() - self.limelight_target_pose.header.stamp
        if dt > rospy.Duration(self.stale_limelight_timeout):
            rospy.logwarn("Not setting localization estimate to limelight. Measurement is too stale.")
            return 
        # ASSUMES limelight target pose is in the same child frame as self.map_to_odom_tf_at_reset
        limelight_target_odom = tf2_geometry_msgs.do_transform_pose(self.limelight_target_pose, self.map_to_odom_tf_at_reset)
        limelight_target_map_at_reset = tf2_geometry_msgs.do_transform_pose(limelight_target_odom, self.map_to_odom_tf_at_reset)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.pose.pose = limelight_target_map_at_reset.pose
        msg.pose.covariance[0] = self.reset_x_cov
        msg.pose.covariance[7] = self.reset_y_cov
        msg.pose.covariance[35] = self.reset_theta_cov
        rospy.loginfo("Resetting localization to limelight target.")

        self.initialpose_pub.publish(msg)
    
    def reset_pose_callback(self, msg):
        self.map_to_odom_tf_at_reset = lookup_transform(self.tf_buffer, self.map_frame, self.odom_frame)
        attempts = 0
        while not rospy.is_shutdown():
            if self.map_to_odom_tf_at_reset is not None:
                break
            rospy.sleep(0.25)
            if attempts > 10:
                break
            attempts += 1
        if self.map_to_odom_tf_at_reset is None:
            rospy.logwarn("Failed to get reset transform")
        else:
            rospy.loginfo("Got reset transform: %s" % self.map_to_odom_tf_at_reset)

    def target_config_callback(self, msg):
        self.enable_shot_correction = msg.enable_shot_correction
        self.enable_shot_probability = msg.enable_shot_probability
        self.enable_limelight_fine_tuning = msg.enable_limelight_fine_tuning
        self.enable_marauding = msg.enable_marauding
        self.enable_reset_to_limelight = msg.enable_reset_to_limelight
        config = {
            "enable_shot_correction": self.enable_shot_correction,
            "enable_shot_probability": self.enable_shot_probability,
            "enable_limelight_fine_tuning": self.enable_limelight_fine_tuning,
            "enable_marauding": self.enable_marauding,
            "enable_reset_to_limelight": self.enable_reset_to_limelight
        }
        config_strs = []
        for key, value in config.items():
            config_strs.append("%s: %s" % (key, value))
        rospy.loginfo("Updating target config: %s" % ", ".join(config_strs))

    def run(self):
        rospy.sleep(1.0)  # wait for waypoints to populate
        clock = rospy.Rate(30.0)
        
        zero_pose_base = PoseStamped()
        zero_pose_base.header.frame_id = self.base_frame
        zero_pose_base.pose.orientation.w = 1.0
        
        while not rospy.is_shutdown():
            shot_probability = 1.0
            if not self.is_global_pose_valid(self.amcl_pose):
                shot_probability = 0.0

            target_waypoint_name = self.target_waypoint
            if self.enable_marauding:
                target_pose_map = self.get_target_waypoint(Pose2d.from_ros_pose(self.robot_pose.pose))
                if target_pose_map is None and target_waypoint_name in self.waypoints:
                    target_pose_map = self.waypoints[target_waypoint_name]
            else:
                if target_waypoint_name in self.waypoints:
                    target_pose_map = self.waypoints[target_waypoint_name]
                else:
                    target_pose_map = None
            if target_pose_map is None:
                shot_probability = 0.0
                self.publish_target(self.target_distance, self.target_heading, shot_probability)
                rospy.logwarn_throttle(1.0, "%s is not an available waypoint" % target_waypoint_name)
                continue
            map_to_target_tf = lookup_transform(self.tf_buffer, self.target_base_frame, self.map_frame)
            if map_to_target_tf is None:
                shot_probability = 0.0
                self.publish_target(self.target_distance, self.target_heading, shot_probability)
                rospy.logwarn_throttle(1.0, "Unable to transfrom from %s -> %s" % (self.target_base_frame, self.map_frame))
                continue
            target_pose = tf2_geometry_msgs.do_transform_pose(target_pose_map, map_to_target_tf)

            base_to_map_tf = lookup_transform(self.tf_buffer, self.map_frame, self.base_frame)
            if base_to_map_tf is None:
                shot_probability = 0.0
                self.publish_target(self.target_distance, self.target_heading, shot_probability)
                rospy.logwarn_throttle(1.0, "Unable to transfrom from %s -> %s" % (self.map_frame, self.base_frame))
                continue
            self.robot_pose = tf2_geometry_msgs.do_transform_pose(zero_pose_base, base_to_map_tf)

            target = Pose2d.from_ros_pose(target_pose.pose)
            target = self.compensate_for_robot_velocity(target)

            if self.enable_shot_probability:
                shot_probability = self.get_shot_probability(target)
            
            if self.enable_limelight_fine_tuning:
                target_pose = self.get_fine_tuned_limelight_target(self.limelight_target_pose, target_pose, self.stale_limelight_timeout, self.limelight_waypoint_agreement_threshold)

            self.target_heading = target.heading() + self.target_cosmic_ray_compensation
            self.target_heading = Pose2d.normalize_theta(self.target_heading + math.pi)
            self.target_distance = target.distance()
            self.publish_target(self.target_distance, self.target_heading, shot_probability)

            target_pose_stamped = PoseStamped()
            target_pose_stamped.header.frame_id = self.target_base_frame
            target_pose_stamped.pose = target.to_ros_pose()
            self.target_pub.publish(target_pose_stamped)

            clock.sleep()
    
    def compensate_for_robot_velocity(self, target: Pose2d, iterations=4):
        if not self.enable_shot_correction:
            return target
        initial_target = Pose2d.from_state(target)
        target = Pose2d.from_state(target)
        for _ in range(iterations):
            if self.hood_state:
                traj_interp = self.traj_interp_up
            else:
                traj_interp = self.traj_interp_down
            tof = self.get_tof(traj_interp, target.distance())
            if tof > 0.0:
                target.x = initial_target.x - self.robot_velocity.x * tof
        return target
    
    def get_tof(self, traj_interp, distance):
        # compute the time of flight of the ball to the target based on a lookup table
        try:
            if traj_interp is None:
                return 0.0
            else:
                return traj_interp(distance)
        except ValueError as e:
            rospy.logwarn_throttle(1.0, "Failed to compute TOF: %s. distance=%0.1fm" % (str(e), distance))
            return -1.0

    def get_shot_probability(self, pose2d: Pose2d):
        # see OccupancyGrid message docs
        # cost is -1 for unknown. Otherwise, 0..100
        # I remap this to 1.0..0.0 for probability (0 == 100% probability, 100 == 0% probability)
        if self.hood_state:
            ogm = self.ogm_up
        else:
            ogm = self.ogm_down
        if ogm is None:
            return 1.0

        try:
            cost = ogm.get_cost_from_world_x_y(pose2d.x, pose2d.y)
        except BaseException as e:
            rospy.logerr_throttle(1.0, "Failed to get probability: %s" % str(e))
            return 0.0
        if cost < 0.0:
            return 0.0
        else:
            return 1.0 - (cost / 100.0)
    
    def get_fine_tuned_limelight_target(self, limelight_pose, waypoint_pose, stale_limelight_s=0.5, agreement_threshold_m=1.0):
        dt = rospy.Time.now() - limelight_pose.header.stamp
        if dt > rospy.Duration(stale_limelight_s):
            return waypoint_pose
        limelight_pose2d = Pose2d.from_ros_pose(limelight_pose.pose)
        waypoint_pose2d = Pose2d.from_ros_pose(waypoint_pose.pose)

        if waypoint_pose2d.distance(limelight_pose2d) < agreement_threshold_m:
            return limelight_pose
        else:
            return waypoint_pose

    def get_target_waypoint(self, robot_pose):
        if rospy.Time.now() - self.color_timestamp > self.color_message_timeout:
            rospy.logwarn_throttle(1.0, "No color sensor message received for at least %s s" % (self.color_message_timeout.to_sec()))
            return None
        if rospy.Time.now() - self.team_timestamp > self.color_message_timeout:
            rospy.logwarn_throttle(1.0, "No team color message received for at least %s s" % (self.color_message_timeout.to_sec()))
            return None
        
        if len(self.loaded_object_color) == 0:
            # If the color sensor doesn't currently see a matching object,
            if rospy.Time.now() - self.cargo_egress_timer > self.cargo_egress_timeout:
                # If the egress timer has expired, assume we want to shoot at the goal
                return None
            # otherwise, use the last detected object name
        else:
            # else use the currently detected object name
            self.cargo_egress_timer = rospy.Time.now()
            self.buffered_object_color = self.loaded_object_color

        if self.team_color == self.buffered_object_color:
            return None
        else:
            marauding_waypoints = []
            team_prefix = self.marauding_target_prefix.replace("<team>", self.team_color)
            for name, waypoint in self.waypoints_pose2d.items():
                if name.startswith(team_prefix):
                    marauding_waypoints.append(waypoint)

            closest_point = min(marauding_waypoints, key=lambda x: x.distance(robot_pose))
            closest_pose = PoseStamped()
            closest_pose.header.frame_id = self.map_frame
            closest_pose.pose = closest_point.to_ros_pose()
            return closest_pose

    def publish_target(self, distance, heading, probability):
        self.nt_pub.publish(self.make_entry("target/distance", distance))
        self.nt_pub.publish(self.make_entry("target/distance_in", meters_to_in(distance)))
        self.nt_pub.publish(self.make_entry("target/heading", heading))
        self.nt_pub.publish(self.make_entry("target/heading_deg", math.degrees(heading)))
        self.nt_pub.publish(self.make_entry("target/probability", probability))
        self.nt_pub.publish(self.make_entry("target/update", rospy.Time.now().to_sec()))

        self.target_angle_pub.publish(Float64(heading))
        self.target_distance_pub.publish(Float64(distance))
        self.target_probability_pub.publish(Float64(probability))

    def is_global_pose_valid(self, amcl_pose):
        if amcl_pose is None or len(amcl_pose.pose.covariance) != 36:
            rospy.logwarn_throttle(1.0, "Invalid AMCL pose! %s" % str(amcl_pose))
            return False

        x_cov = amcl_pose.pose.covariance[0]
        y_cov = amcl_pose.pose.covariance[7]
        theta_cov = amcl_pose.pose.covariance[35]

        if (x_cov < self.x_cov_threshold and 
                y_cov < self.y_cov_threshold and 
                theta_cov < self.theta_cov_threshold):
            return True
        else:
            rospy.logwarn_throttle(1.0, "No valid target. stddev. x=%0.3f, y=%0.3f, theta=%0.3f" % (x_cov, y_cov, theta_cov))
            return False
    
    def make_entry(self, path, value):
        entry = NTEntry()
        entry.path = path
        entry.value = value
        return entry


if __name__ == "__main__":
    node = TJ2Target()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
