#!/usr/bin/env python3
import tf_conversions
import numpy as np
import tf2_ros
import rospy
from threading import Lock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Twist,
    Quaternion,
    TransformStamped,
    PoseStamped,
    Vector3,
)
from typing import List, Optional, Generator, Tuple
from tj2_tools.robot_state import Pose2d, Velocity
from tj2_northstar.filter_model import FilterModel


class TJ2NorthstarFilter:
    def __init__(self) -> None:
        rospy.init_node("tj2_northstar_filter")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.play_forward_buffer_size = rospy.get_param(
            "~play_forward_buffer_size", 100
        )
        self.max_time_window = rospy.get_param("~max_time_window", 0.5)
        self.update_rate = rospy.get_param("~update_rate", 50.0)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.timestamps = np.ascontiguousarray(
            np.zeros(self.play_forward_buffer_size, dtype=np.float64)
        )
        self.odom_messages: List[Optional[Odometry]] = [
            None for _ in range(self.play_forward_buffer_size)
        ]
        self.current_index = 0
        self.base_frame = ""

        self.model = FilterModel(1.0 / self.update_rate)
        self.model_lock = Lock()

        self.odom_sub = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=10
        )
        self.landmark_sub = rospy.Subscriber(
            "landmark", PoseWithCovarianceStamped, self.landmark_callback, queue_size=10
        )

    def find_nearest_index(self, timestamp: float) -> int:
        return np.argmin(np.abs(self.timestamps - timestamp))

    def iterate_forward(
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

    def odom_callback(self, msg: Odometry) -> None:
        if len(self.base_frame) == 0:
            self.base_frame = msg.child_frame_id
        if msg.child_frame_id != self.base_frame:
            raise ValueError(
                "Odometry child frame is inconsistent "
                f"{msg.child_frame_id} != {self.base_frame}"
            )
        self.odom_messages[self.current_index] = msg
        self.timestamps[self.current_index] = msg.header.stamp.to_sec()
        self.current_index = (self.current_index + 1) % self.play_forward_buffer_size
        with self.model_lock:
            self.model.update_odometry(msg)

    def project_twist_forward(
        self, pose2d: Pose2d, twist: Twist, time_delta: float
    ) -> Pose2d:
        velocity = Velocity.from_ros_twist(twist)
        return pose2d.project(velocity, time_delta)

    def get_rpy(self, quaternion: Quaternion):
        return tf_conversions.transformations.euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )

    def publish_transform(self, pose: PoseStamped, child_frame: str) -> None:
        transform = TransformStamped()
        transform.header.stamp = pose.header.stamp
        transform.header.frame_id = pose.header.frame_id
        transform.child_frame_id = child_frame
        transform.transform.translation = Vector3(
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        )
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def landmark_callback(self, msg: PoseWithCovarianceStamped) -> None:
        landmark_timestamp = msg.header.stamp.to_sec()
        landmark_pose2d = Pose2d.from_ros_pose(msg.pose.pose)
        start_pose2d = Pose2d.from_state(landmark_pose2d)

        num_forwards = 0
        for odom_msg, next_timestamp in self.iterate_forward(landmark_timestamp):
            if odom_msg is None:
                continue
            time_delta = next_timestamp - odom_msg.header.stamp.to_sec()
            if time_delta < 0.0:
                continue
            landmark_pose2d = self.project_twist_forward(
                landmark_pose2d, odom_msg.twist.twist, time_delta
            )
            num_forwards += 1
        rospy.loginfo(
            f"Time delay: {rospy.Time.now().to_sec() - landmark_timestamp}. "
            f"Num forwards: {num_forwards}. "
            f"Delta distance: {landmark_pose2d.distance(start_pose2d)}"
        )
        # Apply 3D states not calculated from odom
        roll, pitch, yaw = self.get_rpy(msg.pose.pose.orientation)
        msg.pose.pose.position.x = landmark_pose2d.x
        msg.pose.pose.position.y = landmark_pose2d.y
        msg.pose.pose.orientation = Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(
                roll, pitch, landmark_pose2d.theta
            )
        )
        with self.model_lock:
            self.model.update_landmark(msg)

    def run(self) -> None:
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            rate.sleep()
            with self.model_lock:
                self.model.predict()


def main():
    node = TJ2NorthstarFilter()
    node.run()


if __name__ == "__main__":
    main()
