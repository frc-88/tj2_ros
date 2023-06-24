#!/usr/bin/env python3
from tf_conversions import transformations
import numpy as np
import tf2_ros
import rospy
from threading import Lock

from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
    PoseStamped,
    Vector3,
    Pose,
    Point,
)

from filter_models import TagFastForward
from filter_models import DriveKalmanModel as FilterModel
from helpers import amcl_and_landmark_agree
from tj2_tools.robot_state import Pose2d, Velocity


class TJ2NorthstarFilter:
    def __init__(self) -> None:
        rospy.init_node("tj2_northstar_filter")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = ""
        self.base_frame = ""

        self.play_forward_buffer_size = rospy.get_param("~play_forward_buffer_size", 20)
        self.tag_fast_forward_sample_window = rospy.get_param(
            "~tag_fast_forward_sample_window", 0.1
        )
        self.update_rate = rospy.get_param("~update_rate", 50.0)
        self.roll_pitch_threshold = rospy.get_param("~roll_pitch_threshold", 0.2)
        self.ground_distance_threshold = rospy.get_param(
            "~ground_distance_threshold", 0.5
        )
        self.ground_angle_threshold = rospy.get_param("~ground_angle_threshold", 0.5)

        self.prev_odom = Odometry()

        self.model = FilterModel(1.0 / self.update_rate)
        self.model_lock = Lock()

        self.fast_forwarder = TagFastForward(
            self.tag_fast_forward_sample_window, self.play_forward_buffer_size
        )

        self.amcl_pose = PoseWithCovarianceStamped()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.odom_sub = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=10
        )
        self.landmark_sub = rospy.Subscriber(
            "landmark", PoseWithCovarianceStamped, self.landmark_callback, queue_size=10
        )
        self.amcl_pose_sub = rospy.Subscriber(
            "amcl_pose",
            PoseWithCovarianceStamped,
            self.amcl_pose_callback,
            queue_size=10,
        )
        self.forwarded_landmark_pub = rospy.Publisher(
            "landmark/forwarded", PoseWithCovarianceStamped, queue_size=10
        )
        self.filter_state_pub = rospy.Publisher("filter_state", Odometry, queue_size=10)

    def get_last_pose(self) -> Pose:
        return self.prev_odom.pose.pose

    def odom_callback(self, msg: Odometry) -> None:
        if len(self.base_frame) == 0:
            self.base_frame = msg.child_frame_id
        if len(self.odom_frame) == 0:
            self.odom_frame = msg.header.frame_id
        if msg.child_frame_id != self.base_frame:
            raise ValueError(
                "Odometry child frame is inconsistent "
                f"{msg.child_frame_id} != {self.base_frame}"
            )
        if msg.header.frame_id != self.odom_frame:
            raise ValueError(
                "Odometry frame is inconsistent "
                f"{msg.header.frame_id} != {self.odom_frame}"
            )

        self.prev_odom = msg
        with self.model_lock:
            self.model.update_odometry(msg)
            self.fast_forwarder.record_odometry(msg)

    def publish_transform(self, pose: PoseStamped, child_frame: str) -> None:
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = pose.header.frame_id
        transform.child_frame_id = child_frame
        transform.transform.translation = Vector3(
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        )
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_filter_state(
        self, pose: Pose2d, velocity: Velocity, covariance: np.ndarray
    ) -> None:
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose = pose.to_ros_pose()
        msg.twist.twist = velocity.to_ros_twist()

        pose_covariance = np.zeros((6, 6))
        pose_covariance[0, 0] = covariance[0, 0]
        pose_covariance[1, 1] = covariance[1, 1]
        pose_covariance[5, 5] = covariance[2, 2]
        msg.pose.covariance = pose_covariance.flatten().tolist()

        twist_covariance = np.zeros((6, 6))
        twist_covariance[0, 0] = covariance[3, 3]
        twist_covariance[1, 1] = covariance[4, 4]
        twist_covariance[5, 5] = covariance[5, 5]
        msg.twist.covariance = twist_covariance.flatten().tolist()

        self.filter_state_pub.publish(msg)

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

        mat = transformations.concatenate_matrices(
            transformations.translation_matrix(translation),
            transformations.quaternion_matrix(rotation),
        )
        return mat

    def get_reverse_transform_mat(self, pose: Pose) -> np.ndarray:
        return transformations.inverse_matrix(self.get_forward_transform_mat(pose))

    def landmark_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if len(self.amcl_pose.header.frame_id) == 0:
            rospy.logwarn("AMCL pose not set. Not updating landmark")
            return
        if not amcl_and_landmark_agree(
            self.amcl_pose,
            msg,
            self.roll_pitch_threshold,
            self.ground_distance_threshold,
            self.ground_angle_threshold,
        ):
            rospy.logwarn("AMCL pose do not agree. Not updating landmark")
            return
        if forwarded := self.fast_forwarder.fast_forward(msg):
            self.forwarded_landmark_pub.publish(forwarded)
            with self.model_lock:
                self.model.update_landmark(forwarded)

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = msg
        with self.model_lock:
            self.model.update_landmark(msg)

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
                self.publish_filter_state(
                    self.model.get_pose(),
                    self.model.get_velocity(),
                    self.model.get_covariance(),
                )
                global_pose = self.model.get_pose()
            odom_pose = self.get_last_pose()
            if odom_pose is not None:
                tf_pose = PoseStamped()
                tf_pose.header.frame_id = self.map_frame
                tf_pose.pose = self.get_map_to_odom(
                    global_pose.to_ros_pose(), odom_pose
                )
                self.publish_transform(tf_pose, self.odom_frame)


def main():
    node = TJ2NorthstarFilter()
    node.run()


if __name__ == "__main__":
    main()
