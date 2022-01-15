#!/usr/bin/env python3

import math
import rospy
import tf2_ros

import numpy as np

from vision_msgs.msg import Detection2DArray

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped

from tj2_tools.particle_filter import FilterSerial
from tj2_tools.particle_filter import JitParticleFilter as ParticleFilter
# from tj2_tools.particle_filter import ParticleFilter
from tj2_tools.particle_filter.state import InputVector, State


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
        self.max_objects = rospy.get_param("~max_objects", None)
        self.labels = rospy.get_param("~labels", None)
        self.filter_frame = rospy.get_param("~filter_frame", "base_link")

        assert self.u_std is not None
        assert self.initial_range is not None
        assert self.max_objects is not None
        assert self.labels is not None
        
        self.pfs = {}
        self.inputs = {}
        for label, count in self.max_objects.items():
            self.pfs[label] = []
            self.inputs[label] = []
            for index in range(count):
                self.pfs[label].append(
                    ParticleFilter(
                        FilterSerial(label, index),
                        self.num_particles,
                        self.meas_std_val,
                        self.u_std,
                        self.stale_filter_time
                    )
                )
                self.inputs[label].append(InputVector(self.stale_filter_time))

        self.detections_sub = rospy.Subscriber("detections", Detection2DArray, self.detections_callback, queue_size=25)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=25)

        self.particles_pub = rospy.Publisher("pf_particles", PoseArray, queue_size=5)

        self.broadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo("%s init done" % self.node_name)

    def to_label(self, obj_id):
        return self.labels[obj_id]
        
    def detections_callback(self, msg):
        measurements = {}
        obj_count = {}
        for detection in msg.detections:
            if detection.header.frame_id != self.filter_frame:
                rospy.logwarn_throttle(1.0, "Detection frame does not match filter's frame: %s != %s" % (detection.header.frame_id, self.filter_frame))
            state = State.from_detect(detection)
            label = self.to_label(detection.results[0].id)
            if label not in obj_count:
                obj_count[label] = 0
            index = obj_count[label]
            self.inputs[label][index].meas_update(state)
            pf = self.pfs[label][index]

            meas_z = np.array([state.x, state.y, state.z, state.vx, state.vy, state.vz])
            if not pf.is_initialized() or pf.is_stale():
                rospy.loginfo("initializing with %s" % state)
                pf.create_uniform_particles(meas_z, self.initial_range)
            pf.update(meas_z)

            obj_count[label] += 1

    def odom_callback(self, msg):
        state = State.from_odom(msg)
        for label, index in self.iter_serials():
            pf = self.pfs[label][index]
            if not pf.is_initialized() or pf.is_stale():
                continue
            input_u = self.inputs[label][index]
            dt = input_u.odom_update(state)
            vector = input_u.get_vector()
            pf.predict(vector, dt)

    def iter_serials(self):
        for label in self.pfs.keys():
            for index in range(len(self.pfs[label])):
                yield label, index

    def publish_all_poses(self):
        for label, index in self.iter_serials():
            pf = self.pfs[label][index]
            mean = pf.mean()

            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.filter_frame
            msg.child_frame_id = pf.get_name()
            msg.transform.translation.x = mean[0]
            msg.transform.translation.y = mean[1]
            msg.transform.translation.z = mean[2]
            msg.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(msg)
    
    def publish_particles(self):
        if self.particles_pub.get_num_connections() == 0:
            return
        particles_msg = PoseArray()
        particles_msg.header.frame_id = self.filter_frame
        particles_msg.header.stamp = rospy.Time.now()

        for label, index in self.iter_serials():
            pf = self.pfs[label][index]
            for particle in pf.particles:
                pose_msg = Pose()
                pose_msg.position.x = particle[0]
                pose_msg.position.y = particle[1]
                pose_msg.position.z = particle[2]
                particles_msg.poses.append(pose_msg)

        self.particles_pub.publish(particles_msg)

    def run(self):
        rate = rospy.Rate(30.0)

        while True:
            rate.sleep()
            if rospy.is_shutdown():
                break

            for label, index in self.iter_serials():
                pf = self.pfs[label][index]
                if not pf.is_initialized() or pf.is_stale():
                    continue
                pf.check_resample()

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
