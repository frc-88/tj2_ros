#!/usr/bin/env python3

from typing import List, Optional
import rospy

from dynamic_reconfigure.server import Server

import tf2_ros
import numpy as np

import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler

from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion

from particle_filter import JitParticleFilter
from particle_filter import ParticleFilter

from tj2_interfaces.msg import WaypointArray
from tj2_tools.waypoint import Waypoints2dArray
from tj2_tools.robot_state import Pose2d
from tj2_charged_up.cfg import ParticleFilterConfig


class TagLocalizationNode:
    def __init__(self):
        self.node_name = "tag_localization"
        rospy.init_node(
            self.node_name,
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.meas_std_val = rospy.get_param("~meas_std_val", 0.03)
        self.u_std = rospy.get_param("~u_std", [1.0, 1.0, 1.0])
        self.initial_range = rospy.get_param("~initial_range", [1.0, 1.0, 2 * np.pi])
        self.num_particles = rospy.get_param("~num_particles", 50)
        self.initial_distribution_type = rospy.get_param("~initial_distribution_type", "gaussian")
        self.velocity_smooth_k = rospy.get_param("~velocity_smooth_k", 0.0)
        self.loop_rate = rospy.get_param("~loop_rate", 15.0)
        self.stale_detection_seconds = rospy.Duration(rospy.get_param("~stale_detection_seconds", 1.0))
        self.particle_filter_type = rospy.get_param("~particle_filter_type", "JitParticleFilter")
        self.tag_id_to_waypoint_map = rospy.get_param("~tag_id_to_waypoint_map", {})
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.publish_tf = rospy.get_param("~publish_tf", False)

        if self.particle_filter_type == "JitParticleFilter":
            self.pf = JitParticleFilter(self.num_particles, self.meas_std_val, self.u_std)
        elif self.particle_filter_type == "ParticleFilter":
            self.pf = ParticleFilter(self.num_particles, self.meas_std_val, self.u_std)
        else:
            raise RuntimeError(f"Invalid particle filter type: {self.particle_filter_type}")
        self.pf.initialize_particles(self.u_std, self.initial_distribution_type)
        
        self.prev_predict_time = rospy.Time.now()
        self.waypoints = Waypoints2dArray()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_publisher = rospy.Publisher("robot_tag_pose", PoseStamped, queue_size=5)
        self.detections_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.detections_callback, queue_size=25)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=25)
        self.waypoints_sub = rospy.Subscriber("waypoints", WaypointArray, self.waypoints_callback, queue_size=5)

        self.particles_pub = rospy.Publisher("robot_tag_particles", PoseArray, queue_size=5)

        self.dyn_config = {}
        self.dyn_server = Server(ParticleFilterConfig, self.dyn_config_callback)

        rospy.loginfo("%s init done" % self.node_name)

    def dyn_config_callback(self, config, level):
        rospy.loginfo("Updating particle filter dynamic config")
        if len(self.dyn_config) == 0:
            config["meas_std_val"] = self.meas_std_val
            config["u_std_vx"] = self.u_std[0]
            config["u_std_vy"] = self.u_std[1]
            config["u_std_vt"] = self.u_std[2]
            config["num_particles"] = self.num_particles
            config["initial_distribution_type"] = self.initial_distribution_type
            self.dyn_config = config
            return self.dyn_config

        self.meas_std_val = self.get_default_config("meas_std_val", config, self.meas_std_val)
        self.u_std = [
            self.get_default_config("u_std_vx", config, self.u_std[0]),
            self.get_default_config("u_std_vy", config, self.u_std[1]),
            self.get_default_config("u_std_vt", config, self.u_std[2]),
        ]
        self.num_particles = self.get_default_config("num_particles", config, self.num_particles)
        self.initial_distribution_type = self.get_default_config("initial_distribution_type", config, self.initial_distribution_type)
        
        self.pf.set_parameters(self.num_particles, self.meas_std_val, self.u_std)
        self.pf.initialize_particles(self.u_std, self.initial_distribution_type)

        return self.dyn_config
    
    def get_default_config(self, key, new_config, default):
        if self.dyn_config[key] != new_config[key]:
            self.dyn_config[key] = new_config[key]
            rospy.loginfo("Setting %s to %s" % (key, new_config[key]))
            return new_config[key]
        else:
            return default

    def waypoints_callback(self, msg: WaypointArray):
        self.waypoints = Waypoints2dArray.from_waypoints_array(msg)

    def detections_callback(self, msg):
        states = []
        for detection in msg.detections:
            state = self.tag_to_robot_state(detection)
            if state is None:
                continue
            states.append(state)
        weighted_state = np.mean(np.array(states, dtype=np.float64))
        self.pf.update(weighted_state)

    def tag_to_robot_state(self, detection: AprilTagDetection) -> Optional[np.ndarray]:
        # convert detection to pose stamped
        # lookup tag id in waypoint map
        # request global waypoint pose
        # apply robot->tag transform relative to waypoint. get pose
        # convert pose to particle filter measurement

        tag_pose = PoseStamped()
        tag_pose.header = detection.pose.header
        tag_pose.pose = detection.pose.pose.pose
        tag_base_pose = self.transform_tag_to_base(tag_pose)
        if tag_base_pose is None:
            return None
        tag_base_pose_2d = Pose2d.from_ros_pose(tag_base_pose.pose)
        
        tag_id: List[int] = detection.id
        name = "-".join([str(sub_id) for sub_id in tag_id])
        
        if name not in self.tag_id_to_waypoint_map:
            rospy.logwarn(f"Tag {name} is not mapped to a waypoint. Ignoring.")
            return None
        waypoint_name = self.tag_id_to_waypoint_map[name]
        waypoint = self.waypoints.get(waypoint_name)
        if waypoint is None:
            rospy.logwarn(f"Waypoint {waypoint_name} is not a valid waypoint. Ignoring.")
            return None
        robot_global_pose2d = waypoint.transform_by(tag_base_pose_2d)
        return np.array(robot_global_pose2d.to_list())

    def transform_tag_to_base(self, tag_pose_stamped: PoseStamped) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                tag_pose_stamped.header.frame_id,
                rospy.Time(0),
                self.stale_detection_seconds
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (self.robot_frame, tag_pose_stamped.header.frame_id, e))
            return None
        return tf2_geometry_msgs.do_transform_pose(tag_pose_stamped, transform)

    def odom_callback(self, msg):
        dt = self.get_predict_dt(msg.header.stamp)
        self.pf.predict(self.odom_to_predict_vector(msg), dt)

    def get_predict_dt(self, stamp) -> float:
        # return time since last predict message
        dt = (stamp - self.prev_predict_time).to_sec()
        self.prev_predict_time = stamp
        return dt

    def odom_to_predict_vector(self, msg: Odometry) -> np.ndarray:
        # assumes velocities are in the robot base_link frame
        return np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        ])

    def publish_all_pose(self):
        pose = self.robot_state_to_pose(self.pf.mean())
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = self.global_frame
        pose_stamped.pose = pose
        self.pose_publisher.publish(pose_stamped)
    
    def robot_state_to_pose(self, state: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x = state[0]
        pose.position.y = state[1]
        pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, state[2]))
        return pose
    
    def publish_particles(self):
        if self.particles_pub.get_num_connections() == 0:
            return
        particles_msg = PoseArray()
        particles_msg.header.frame_id = self.global_frame
        particles_msg.header.stamp = rospy.Time.now()

        for particle in self.pf.particles:
            particles_msg.poses.append(self.robot_state_to_pose(particle))

        self.particles_pub.publish(particles_msg)

    def run(self):
        rate = rospy.Rate(self.loop_rate)

        while True:
            rate.sleep()
            if rospy.is_shutdown():
                break

            if self.pf.is_initialized():
                self.pf.check_resample()

            self.publish_all_pose()
            self.publish_particles()


if __name__ == "__main__":
    node = TagLocalizationNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
