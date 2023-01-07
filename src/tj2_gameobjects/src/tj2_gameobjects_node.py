#!/usr/bin/env python3

import math
import rospy

from dynamic_reconfigure.server import Server

import numpy as np

from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection3DArray

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped

from tj2_tools.particle_filter import FilterSerial
from tj2_tools.particle_filter import JitParticleFilter as ParticleFilter
# from tj2_tools.particle_filter import ParticleFilter
from tj2_tools.particle_filter.state import InputVector, Simple3DState
from tj2_tools.particle_filter.predictor import BouncePredictor

from tj2_gameobjects.cfg import ParticleFilterConfig


class Tj2GameobjectsNode:
    def __init__(self):
        self.node_name = "tj2_gameobjects"
        rospy.init_node(
            self.node_name,
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.meas_std_val = rospy.get_param("~meas_std_val", 0.03)
        self.u_std = rospy.get_param("~u_std", None)
        self.initial_range = rospy.get_param("~initial_range", None)
        self.num_particles = rospy.get_param("~num_particles", 50)
        self.stale_filter_time = rospy.get_param("~stale_filter_time", 1.0)
        self.labels = rospy.get_param("~labels", None)
        self.bounds = rospy.get_param("~bounds", None)
        self.filter_frame = rospy.get_param("~filter_frame", "base_link")
        self.use_3d_detections = rospy.get_param("~use_3d_detections", True)
        self.velocity_smooth_k = rospy.get_param("~velocity_smooth_k", 0.0)

        assert self.u_std is not None
        assert self.initial_range is not None
        assert self.labels is not None
        assert self.bounds is not None

        self.pfs = {}
        self.predictors = {}
        self.inputs = {}
        self.pose_publishers = {}
        self.future_publishers = {}
        for label in self.labels:
            serial = FilterSerial(label, 0)
            self.pfs[serial] = ParticleFilter(
                        serial,
                        self.num_particles,
                        self.meas_std_val,
                        self.u_std,
                        self.stale_filter_time,
                        self.bounds
                    )
            self.inputs[serial] = InputVector(self.stale_filter_time, self.velocity_smooth_k)
            self.predictors[serial] = BouncePredictor(  # TODO: make dynamically configurable
                rho=0.75,
                tau=0.05,
                g=-9.81,
                a_friction=-0.1,
                t_step=0.001,
                ground_plane=self.bounds[2][0]
            )
            self.pose_publishers[serial] = rospy.Publisher("gameobject/estimate_%s_%s" % (serial.label, serial.index), PoseStamped, queue_size=5)
            self.future_publishers[serial] = rospy.Publisher("gameobject/future_%s_%s" % (serial.label, serial.index), PoseStamped, queue_size=5)
        detection_type = Detection3DArray if self.use_3d_detections else Detection2DArray
        self.detections_sub = rospy.Subscriber("detections", detection_type, self.detections_callback, queue_size=25)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=25)

        self.particles_pub = rospy.Publisher("pf_particles", PoseArray, queue_size=5)

        self.dyn_config = {}
        self.dyn_server = Server(ParticleFilterConfig, self.dyn_config_callback)

        rospy.loginfo("%s init done" % self.node_name)

    def dyn_config_callback(self, config, level):
        rospy.loginfo("Updating particle filter dynamic config")
        if len(self.dyn_config) == 0:
            config["meas_std_val"] = self.meas_std_val
            config["u_std_vx"] = self.u_std[0]
            config["u_std_vy"] = self.u_std[1]
            config["u_std_vz"] = self.u_std[2]
            config["u_std_vt"] = self.u_std[3]
            config["initial_range_x"] = self.initial_range[0]
            config["initial_range_y"] = self.initial_range[1]
            config["initial_range_z"] = self.initial_range[2]
            config["initial_range_vx"] = self.initial_range[3]
            config["initial_range_vy"] = self.initial_range[4]
            config["initial_range_vz"] = self.initial_range[5]
            config["num_particles"] = self.num_particles
            config["stale_filter_time"] = self.stale_filter_time
            config["lower_bound_x"] = self.bounds[0][0]
            config["upper_bound_x"] = self.bounds[0][1]
            config["lower_bound_y"] = self.bounds[1][0]
            config["upper_bound_y"] = self.bounds[1][1]
            config["lower_bound_z"] = self.bounds[2][0]
            config["upper_bound_z"] = self.bounds[2][1]
            config["lower_bound_vx"] = self.bounds[3][0]
            config["upper_bound_vx"] = self.bounds[3][1]
            config["lower_bound_vy"] = self.bounds[4][0]
            config["upper_bound_vy"] = self.bounds[4][1]
            config["lower_bound_vz"] = self.bounds[5][0]
            config["upper_bound_vz"] = self.bounds[5][1]
            config["velocity_smooth_k"] = self.velocity_smooth_k
            self.dyn_config = config
            return self.dyn_config

        self.meas_std_val = self.get_default_config("meas_std_val", config, self.meas_std_val)
        self.u_std = [
            self.get_default_config("u_std_vx", config, self.u_std[0]),
            self.get_default_config("u_std_vy", config, self.u_std[1]),
            self.get_default_config("u_std_vz", config, self.u_std[2]),
            self.get_default_config("u_std_vt", config, self.u_std[3]),
        ]
        self.initial_range = [
            self.get_default_config("initial_range_x", config, self.initial_range[0]),
            self.get_default_config("initial_range_y", config, self.initial_range[1]),
            self.get_default_config("initial_range_z", config, self.initial_range[2]),
            self.get_default_config("initial_range_vx", config, self.initial_range[3]),
            self.get_default_config("initial_range_vy", config, self.initial_range[4]),
            self.get_default_config("initial_range_vz", config, self.initial_range[5]),
        ]
        self.num_particles = self.get_default_config("num_particles", config, self.num_particles)
        self.stale_filter_time = self.get_default_config("stale_filter_time", config, self.stale_filter_time)

        self.bounds = [
            [
                self.get_default_config("lower_bound_x", config, self.bounds[0][0]),
                self.get_default_config("upper_bound_x", config, self.bounds[0][1]),
            ],
            [
                self.get_default_config("lower_bound_y", config, self.bounds[1][0]),
                self.get_default_config("upper_bound_y", config, self.bounds[1][1]),
            ],
            [
                self.get_default_config("lower_bound_z", config, self.bounds[2][0]),
                self.get_default_config("upper_bound_z", config, self.bounds[2][1]),
            ],
            [
                self.get_default_config("lower_bound_vx", config, self.bounds[3][0]),
                self.get_default_config("upper_bound_vx", config, self.bounds[3][1]),
            ],
            [
                self.get_default_config("lower_bound_vy", config, self.bounds[4][0]),
                self.get_default_config("upper_bound_vy", config, self.bounds[4][1]),
            ],
            [
                self.get_default_config("lower_bound_vz", config, self.bounds[5][0]),
                self.get_default_config("upper_bound_vz", config, self.bounds[5][1]),
            ],
        ]
        self.velocity_smooth_k = self.get_default_config("velocity_smooth_k", config, self.velocity_smooth_k)
        
        for serial, pf in self.iter_pfs():
            pf.set_parameters(self.num_particles, self.meas_std_val, self.u_std, self.stale_filter_time)
            self.inputs[serial].set_smooth_k(self.velocity_smooth_k)

        return self.dyn_config
    
    def get_default_config(self, key, new_config, default):
        if self.dyn_config[key] != new_config[key]:
            self.dyn_config[key] = new_config[key]
            rospy.loginfo("Setting %s to %s" % (key, new_config[key]))
            return new_config[key]
        else:
            return default

    def to_label(self, obj_id):
        if self.use_3d_detections:
            class_index = obj_id & 0xffff
            # class_count = obj_id >> 16
            return self.labels[class_index]
        else:
            return self.labels[obj_id - 1]  # BACKGROUND = 0, but we ignore it here
        
    def detections_callback(self, msg):
        measurements = {}
        for detection in msg.detections:
            if detection.header.frame_id != self.filter_frame:
                rospy.logwarn_throttle(1.0, "Detection frame does not match filter's frame: %s != %s" % (detection.header.frame_id, self.filter_frame))
            state = Simple3DState.from_detect(detection)

            label = self.to_label(detection.results[0].id)
            if label not in measurements:
                measurements[label] = [state]
            else:
                measurements[label].append(state)

        for label, states in measurements.items():
            max_dist = 0.0
            max_index = 0
            for index, state in enumerate(states):
                distance = state.distance()
                if distance > max_dist:
                    max_dist = distance
                    max_index = index
            state = states[max_index]
            
            serial = FilterSerial(label, 0)  # one object per label type
            pf = self.pfs[serial]

            meas_z = np.array([state.x, state.y, state.z, state.vx, state.vy, state.vz])
            if not pf.is_initialized() or pf.is_stale():
                rospy.loginfo("initializing with %s" % state)
                pf.create_uniform_particles(meas_z, self.initial_range)
            
            self.inputs[serial].meas_update(state)
            pf.update(meas_z)

    def odom_callback(self, msg):
        state = Simple3DState.from_odom(msg)
        for serial, pf in self.iter_pfs():
            if not pf.is_initialized():  #  or pf.is_stale():
                continue
            input_u = self.inputs[serial]
            dt = input_u.odom_update(state)
            vector = input_u.get_vector()
            pf.predict(vector, dt)

    def iter_pfs(self):
        for serial, pf in self.pfs.items():
            yield serial, pf

    def predict_trajectories(self):
        for serial, pf in self.iter_pfs():
            input_u = self.inputs[serial]
            predictor = self.predictors[serial]
            pf_state = pf.get_state()
            static_frame_state = pf_state.relative_to(input_u.odom_state)

            future_state_odom = predictor.get_robot_intersection(input_u.odom_state, static_frame_state)
            future_state_base_link = future_state_odom.relative_to_reverse(input_u.odom_state)

            future_pose = future_state_base_link.to_ros_pose()
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.filter_frame
            msg.pose = future_pose
            self.future_publishers[serial].publish(msg)

    def publish_all_poses(self):
        for serial, pf in self.iter_pfs():
            mean = pf.mean()

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.filter_frame
            msg.pose.position.x = mean[0]
            msg.pose.position.y = mean[1]
            msg.pose.position.z = mean[2]
            msg.pose.orientation.w = 1.0
            self.pose_publishers[serial].publish(msg)
    
    def publish_particles(self):
        if self.particles_pub.get_num_connections() == 0:
            return
        particles_msg = PoseArray()
        particles_msg.header.frame_id = self.filter_frame
        particles_msg.header.stamp = rospy.Time.now()

        for serial, pf in self.iter_pfs():
            for particle in pf.particles:
                pose_msg = Pose()
                pose_msg.position.x = particle[0]
                pose_msg.position.y = particle[1]
                pose_msg.position.z = particle[2]
                particles_msg.poses.append(pose_msg)

        self.particles_pub.publish(particles_msg)

    def run(self):
        rate = rospy.Rate(60.0)

        while True:
            rate.sleep()
            if rospy.is_shutdown():
                break

            for serial, pf in self.iter_pfs():
                if not pf.is_initialized():  # or pf.is_stale():
                    continue
                pf.check_resample()
            
            self.predict_trajectories()

            self.publish_all_poses()
            self.publish_particles()


if __name__ == "__main__":
    node = Tj2GameobjectsNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
