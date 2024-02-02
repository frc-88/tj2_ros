#!/usr/bin/env python3
from threading import Lock

import numpy as np
import rospy
import tf2_ros
from filter_models import DriveKalmanModel as FilterModel
from filter_models import TagFastForward
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
    TwistWithCovariance,
    Vector3,
)
from helpers import amcl_and_landmark_agree, is_roll_pitch_reasonable
from nav_msgs.msg import Odometry
from tf_conversions import transformations


class TJ2NorthstarFilter:
    def __init__(self) -> None:
        rospy.init_node("tj2_northstar_filter")
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.odom_frame = ""  # Will be set by the first odometry message
        self.base_frame = ""  # Will be set by the first odometry message

        self.play_forward_buffer_size = int(rospy.get_param("~play_forward_buffer_size", 20))
        self.tag_fast_forward_sample_dt = float(rospy.get_param("~tag_fast_forward_sample_dt", 0.05))
        self.update_rate = float(rospy.get_param("~update_rate", 50.0))
        self.roll_pitch_threshold = float(rospy.get_param("~roll_pitch_threshold", 0.2))
        self.ground_distance_threshold = float(rospy.get_param("~ground_distance_threshold", 0.5))
        self.ground_angle_threshold = float(rospy.get_param("~ground_angle_threshold", 0.5))
        self.use_amcl = bool(rospy.get_param("~use_amcl", True))

        self.prev_odom = Odometry()

        self.model = FilterModel(1.0 / self.update_rate)
        self.model_lock = Lock()

        self.fast_forwarder = TagFastForward(self.tag_fast_forward_sample_dt, self.play_forward_buffer_size)

        self.amcl_pose = PoseWithCovarianceStamped()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=10)
        self.landmark_sub = rospy.Subscriber(
            "landmark", PoseWithCovarianceStamped, self.landmark_callback, queue_size=10
        )
        self.reset_sub = rospy.Subscriber("reset_pose", PoseWithCovarianceStamped, self.reset_callback, queue_size=10)
        self.amcl_pose_sub = rospy.Subscriber(
            "amcl_pose",
            PoseWithCovarianceStamped,
            self.amcl_pose_callback,
            queue_size=10,
        )
        self.initial_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.forwarded_landmark_pub = rospy.Publisher("landmark/forwarded", PoseWithCovarianceStamped, queue_size=10)
        self.filter_state_pub = rospy.Publisher("filter/state", Odometry, queue_size=10)
        self.filter_pose_pub = rospy.Publisher("filter/pose", PoseWithCovarianceStamped, queue_size=10)

    def get_last_pose(self) -> Pose:
        return self.prev_odom.pose.pose

    def odom_callback(self, msg: Odometry) -> None:
        if len(self.base_frame) == 0:
            self.base_frame = msg.child_frame_id
        if len(self.odom_frame) == 0:
            self.odom_frame = msg.header.frame_id
        if msg.child_frame_id != self.base_frame:
            raise ValueError("Odometry child frame is inconsistent " f"{msg.child_frame_id} != {self.base_frame}")
        if msg.header.frame_id != self.odom_frame:
            raise ValueError("Odometry frame is inconsistent " f"{msg.header.frame_id} != {self.odom_frame}")

        self.prev_odom = msg
        with self.model_lock:
            self.model.update_cmd_vel(msg.twist)
            self.fast_forwarder.record_odometry(msg)

    def publish_transform(self, pose: PoseStamped, child_frame: str) -> None:
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = pose.header.frame_id
        transform.child_frame_id = child_frame
        transform.transform.translation = Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_filter_state(self, pose: PoseWithCovariance, twist: TwistWithCovariance) -> None:
        state_msg = Odometry()
        state_msg.header.frame_id = self.map_frame
        state_msg.header.stamp = rospy.Time.now()
        state_msg.child_frame_id = self.base_frame
        state_msg.pose = pose
        state_msg.twist = twist

        self.filter_state_pub.publish(state_msg)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = state_msg.header
        pose_msg.pose = pose
        self.filter_pose_pub.publish(pose_msg)

    def get_map_to_odom(self, map_to_base_pose: Pose, odom_to_base_pose: Pose) -> Pose:
        base2map = self.get_reverse_transform_mat(map_to_base_pose)
        odom2base = self.get_forward_transform_mat(odom_to_base_pose)
        map2odom = transformations.inverse_matrix(odom2base @ base2map)
        return self.get_pose_from_mat(map2odom)

    def get_pose_from_mat(self, mat: np.ndarray) -> Pose:
        pose = Pose()
        pose.position = Point(*transformations.translation_from_matrix(mat))
        pose.orientation = Quaternion(*transformations.quaternion_from_matrix(mat))
        return pose

    def get_forward_transform_mat(self, pose: Pose) -> np.ndarray:
        translation = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        rotation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        mat = np.array(
            transformations.concatenate_matrices(
                transformations.translation_matrix(translation),
                transformations.quaternion_matrix(rotation),
            )
        )
        return mat

    def get_reverse_transform_mat(self, pose: Pose) -> np.ndarray:
        return transformations.inverse_matrix(self.get_forward_transform_mat(pose))

    def landmark_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if self.use_amcl:
            if len(self.amcl_pose.header.frame_id) == 0:
                rospy.logwarn_throttle(1.0, "AMCL pose not set. Not updating landmark")
                return

            if not is_roll_pitch_reasonable(msg, self.roll_pitch_threshold):
                rospy.loginfo_throttle(1.0, "Rejecting landmark. Roll or pitch is too high")
                return

            if not amcl_and_landmark_agree(
                self.amcl_pose,
                msg,
                self.ground_distance_threshold,
                self.ground_angle_threshold,
            ):
                rospy.logwarn("AMCL pose do not agree. Not updating landmark")
                return
        forwarded = self.fast_forwarder.fast_forward(msg)
        if forwarded is not None:
            self.forwarded_landmark_pub.publish(forwarded)
            with self.model_lock:
                self.model.update_pose(forwarded.pose)

    def reset_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.model.teleport(msg.pose)
        self.initial_pose_pub.publish(msg)

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if self.use_amcl:
            self.amcl_pose = msg
            with self.model_lock:
                self.model.update_pose(msg.pose)
        else:
            rospy.logwarn("AMCL pose received but not used")

    def run(self) -> None:
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                continue

            if len(self.odom_frame) == 0 and len(self.base_frame) == 0:
                continue
            with self.model_lock:
                self.model.predict()

                pose, twist = self.model.get_state()
                self.publish_filter_state(pose, twist)
            odom_pose = self.get_last_pose()
            if odom_pose is not None:
                tf_pose = PoseStamped()
                tf_pose.header.frame_id = self.map_frame
                tf_pose.pose = self.get_map_to_odom(pose.pose, odom_pose)
                self.publish_transform(tf_pose, self.odom_frame)


def main():
    node = TJ2NorthstarFilter()
    node.run()


if __name__ == "__main__":
    main()
