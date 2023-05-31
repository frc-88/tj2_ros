#!/usr/bin/env python3
from tf_conversions import transformations
import numpy as np
import tf2_ros
import rospy
from typing import List, Optional, Generator, Tuple
from threading import Lock

from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Twist,
    Quaternion,
    TransformStamped,
    PoseStamped,
    Vector3,
    Pose,
    Point,
)

from tj2_tools.robot_state import Pose2d, Velocity
from tj2_tools.transforms import transform_pose

from filter_model import FilterModel


class TJ2NorthstarFilter:
    def __init__(self) -> None:
        rospy.init_node("tj2_northstar_filter")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.play_forward_buffer_size = rospy.get_param("~play_forward_buffer_size", 20)
        self.max_time_window = rospy.get_param("~max_time_window", 0.5)
        self.update_rate = rospy.get_param("~update_rate", 50.0)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.timestamps = np.ascontiguousarray(
            np.zeros(self.play_forward_buffer_size, dtype=np.float64)
        )
        self.odom_messages: List[Optional[Odometry]] = [
            None for _ in range(self.play_forward_buffer_size)
        ]
        self.prev_odom = Odometry()
        self.current_index = 0

        self.buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.buffer)

        self.model = FilterModel(1.0 / self.update_rate)
        self.model_lock = Lock()

        self.odom_sub = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=10
        )
        self.landmark_sub = rospy.Subscriber(
            "landmark", PoseWithCovarianceStamped, self.landmark_callback, queue_size=10
        )
        self.filter_state_pub = rospy.Publisher("filter_state", Odometry, queue_size=10)

    def find_nearest_index(self, timestamp: float) -> int:
        return np.argmin(np.abs(self.timestamps - timestamp))

    def iterate_odom_forward(
        self, past_timestamp: float
    ) -> Generator[Tuple[Optional[Odometry], float], None, None]:
        nearest_index = self.find_nearest_index(past_timestamp)
        index = nearest_index
        now = rospy.Time.now().to_sec()
        while index != self.current_index:
            next_index = (index + 1) % self.play_forward_buffer_size
            odom_timestamp = self.timestamps[index]
            forward_delta = odom_timestamp - past_timestamp
            lag_delta = now - odom_timestamp
            if forward_delta < 0.0 and lag_delta < self.max_time_window:
                if next_index == index:
                    next_timestamp = now
                else:
                    next_timestamp = self.timestamps[next_index]
                yield self.odom_messages[index], next_timestamp
            index = next_index

    def get_last_odom(self) -> Optional[Pose]:
        # zero_pose = PoseStamped()
        # zero_pose.header.frame_id = self.base_frame
        # zero_pose.pose.orientation.w = 1.0
        # pose_stamped = transform_pose(self.buffer, zero_pose, self.odom_frame)
        # if pose_stamped is None:
        #     return None
        # else:
        #     return pose_stamped.pose

        # odom = self.odom_messages[self.current_index]
        # if odom is None:
        #     return None
        # else:
        #     return odom.pose.pose

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
        self.odom_messages[self.current_index] = msg
        self.timestamps[self.current_index] = msg.header.stamp.to_sec()
        self.current_index = (self.current_index + 1) % self.play_forward_buffer_size
        self.prev_odom = msg
        with self.model_lock:
            self.model.update_odometry(msg)

    def project_twist_forward(
        self, pose2d: Pose2d, twist: Twist, time_delta: float
    ) -> Pose2d:
        velocity = Velocity.from_ros_twist(twist)
        return pose2d.project(velocity, time_delta)

    def get_rpy(self, quaternion: Quaternion):
        return transformations.euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )

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

    def publish_filter_state(self) -> None:
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose = self.model.get_pose().to_ros_pose()
        msg.twist.twist = self.model.get_velocity().to_ros_twist()
        covariance = self.model.get_covariance()

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
        landmark_pose2d = Pose2d.from_ros_pose(msg.pose.pose)

        # landmark_timestamp = msg.header.stamp.to_sec()
        # start_pose2d = Pose2d.from_state(landmark_pose2d)
        # num_forwards = 0
        # for odom_msg, next_timestamp in self.iterate_odom_forward(landmark_timestamp):
        #     if odom_msg is None:
        #         continue
        #     time_delta = next_timestamp - odom_msg.header.stamp.to_sec()
        #     if time_delta < 0.0:
        #         continue
        #     landmark_pose2d = self.project_twist_forward(
        #         landmark_pose2d, odom_msg.twist.twist, time_delta
        #     )
        #     num_forwards += 1
        # rospy.loginfo(
        #     f"Time delay: {rospy.Time.now().to_sec() - landmark_timestamp}. "
        #     f"Num forwards: {num_forwards}. "
        #     f"Delta distance: {landmark_pose2d.distance(start_pose2d)}"
        # )

        # Apply 3D states not calculated from odom
        roll, pitch, yaw = self.get_rpy(msg.pose.pose.orientation)
        msg.pose.pose.position.x = landmark_pose2d.x
        msg.pose.pose.position.y = landmark_pose2d.y
        msg.pose.pose.orientation = Quaternion(
            *transformations.quaternion_from_euler(roll, pitch, landmark_pose2d.theta)
        )
        with self.model_lock:
            self.model.update_landmark(msg)

    def run(self) -> None:
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            rate.sleep()
            with self.model_lock:
                self.model.predict()
                self.publish_filter_state()
                global_pose = self.model.get_pose()
                odom_pose = self.get_last_odom()
                if odom_pose is not None:
                    tf_pose = PoseStamped()
                    tf_pose.header.frame_id = self.map_frame
                    tf_pose.pose = self.get_map_to_odom(
                        global_pose.to_ros_pose(), odom_pose
                    )
                    self.publish_transform(tf_pose, self.odom_frame)
                # self.publish_transform(global_pose.to_ros_pose(), self.base_frame)


def main():
    node = TJ2NorthstarFilter()
    node.run()


if __name__ == "__main__":
    main()
