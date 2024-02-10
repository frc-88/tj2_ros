#!/usr/bin/env python3
import math
from typing import Dict, FrozenSet, List, Optional

import numpy as np
import rospy
import tf.transformations
import tf2_geometry_msgs
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tj2_tools.robot_state import SimpleFilter
from tj2_tools.transforms import lookup_transform

TagBundleId = FrozenSet[int]


class LandmarkConverter:
    def __init__(self) -> None:
        rospy.init_node("landmark_converter")

        self.base_frame = str(rospy.get_param("~base_frame", "base_tilt_link"))
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.max_tag_distance = float(rospy.get_param("~max_tag_distance", 1000.0))
        self.time_covariance_filter_k = float(rospy.get_param("~time_covariance_filter_k", 0.5))
        base_covariance = rospy.get_param("~covariance", np.eye(6, dtype=np.float64).flatten().tolist())
        assert len(base_covariance) == 36, f"Invalid covariance. Length is {len(base_covariance)}: {base_covariance}"
        self.base_covariance = np.array(base_covariance).reshape((6, 6))

        self.prev_landmark = PoseStamped()

        self.time_covariance_filter = SimpleFilter(self.time_covariance_filter_k)

        self.buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.buffer)

        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tags_callback, queue_size=10)
        self.landmark_pub = rospy.Publisher("landmark", PoseWithCovarianceStamped, queue_size=10)

    def tags_callback(self, msg: AprilTagDetectionArray) -> None:
        assert msg.detections is not None
        landmark_poses: Dict[TagBundleId, PoseStamped] = {}
        individual_measurements = []
        for detection in msg.detections:
            ids = frozenset(detection.id)
            detection_pose = PoseStamped()
            detection_pose.header = detection.pose.header
            detection_pose.pose = detection.pose.pose.pose
            if len(ids) == 1:
                individual_measurements.append(detection_pose)
            else:
                landmark_poses[ids] = detection_pose

        for landmark_pose in landmark_poses.values():
            if len(individual_measurements) == 0:
                continue
            if not self.should_publish(individual_measurements):
                continue
            inverted_pose = self.invert_transform_pose(landmark_pose)
            if inverted_pose is None:
                continue

            landmark = PoseWithCovarianceStamped()
            landmark.header = inverted_pose.header
            landmark.pose.pose = inverted_pose.pose
            landmark.pose.covariance = self.get_covariance(individual_measurements, landmark_pose)
            self.landmark_pub.publish(landmark)

    def should_publish(self, tag_poses: List[PoseStamped]) -> bool:
        """
        Should publish if at least one tag is closer than the threshold distance
        """
        if len(tag_poses) <= 1:
            return False
        for tag_pose in tag_poses:
            distance = self.get_distance(tag_pose)
            if distance < self.max_tag_distance:
                return True
        return False

    def get_distance(self, tag_pose: PoseStamped) -> float:
        x = tag_pose.pose.position.x
        y = tag_pose.pose.position.y
        z = tag_pose.pose.position.z
        return math.sqrt(x * x + y * y + z * z)

    def invert_transform_pose(self, pose: PoseStamped) -> Optional[PoseStamped]:
        transform = lookup_transform(self.buffer, self.base_frame, pose.header.frame_id)
        if transform is None:
            return None
        landmark_in_base = tf2_geometry_msgs.do_transform_pose(pose, transform)
        translation = (
            landmark_in_base.pose.position.x,
            landmark_in_base.pose.position.y,
            landmark_in_base.pose.position.z,
        )
        rotation = (
            landmark_in_base.pose.orientation.x,
            landmark_in_base.pose.orientation.y,
            landmark_in_base.pose.orientation.z,
            landmark_in_base.pose.orientation.w,
        )

        inverse_mat = tf.transformations.inverse_matrix(tf.transformations.translation_matrix(translation))

        inverse_mat = tf.transformations.concatenate_matrices(
            tf.transformations.inverse_matrix(tf.transformations.quaternion_matrix(rotation)),
            inverse_mat,
        )

        forward_translation = tf.transformations.translation_from_matrix(inverse_mat)
        forward_rotation = tf.transformations.quaternion_from_matrix(inverse_mat)
        inverse_pose = PoseStamped()
        inverse_pose.header.frame_id = self.map_frame
        inverse_pose.header.stamp = pose.header.stamp
        inverse_pose.pose.position.x = forward_translation[0]
        inverse_pose.pose.position.y = forward_translation[1]
        inverse_pose.pose.position.z = forward_translation[2]
        inverse_pose.pose.orientation.x = forward_rotation[0]
        inverse_pose.pose.orientation.y = forward_rotation[1]
        inverse_pose.pose.orientation.z = forward_rotation[2]
        inverse_pose.pose.orientation.w = forward_rotation[3]
        return inverse_pose

    def num_tags_covariance_scale(self, num_tags: int) -> float:
        if num_tags == 0:
            raise ValueError("Can't compute covariance with no measurements!")
        # 1 -> 5
        elif num_tags == 1:
            scale = 5
        else:
            # 2 -> 0.5
            # 3 -> 0.25
            # 4 -> 0.125
            scale = 1.0 / (2 ** (num_tags - 1))
        return scale

    def pose_distance_covariance_scale(self, distance: float) -> float:
        if distance < 0.5:
            return 10.0
        return 0.25 * distance**2.0

    def get_pose_distance(self, pose: PoseStamped) -> float:
        return float(
            np.linalg.norm(
                [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ]
            )
        )

    def pose_stamped_to_vector(self, pose: PoseStamped) -> np.ndarray:
        return np.array(
            [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ]
        )

    def delta_pose_covariance_scale(self, current_pose: PoseStamped, prev_pose: PoseStamped) -> None:
        distance = np.linalg.norm(self.pose_stamped_to_vector(current_pose) - self.pose_stamped_to_vector(prev_pose))
        scale = 4 * distance + 1.0
        return scale

    def time_covariance_scale(self, current_pose: PoseStamped, prev_pose: PoseStamped) -> float:
        time_delta = (current_pose.header.stamp - prev_pose.header.stamp).to_sec()
        time_delta = max(time_delta, 5.0)
        filtered_delta = self.time_covariance_filter.update(time_delta)
        if filtered_delta < time_delta:
            time_delta = filtered_delta
        return time_delta * 2.0

    def get_covariance(self, tag_poses: List[PoseStamped], overall_pose: PoseStamped) -> List[float]:
        distances = [self.get_pose_distance(pose) for pose in tag_poses]
        aggregate_distance = float(np.median(distances))

        covariance = self.base_covariance * self.num_tags_covariance_scale(len(tag_poses))
        covariance *= self.pose_distance_covariance_scale(aggregate_distance)

        covariance *= self.delta_pose_covariance_scale(overall_pose, self.prev_landmark)
        covariance *= self.time_covariance_scale(overall_pose, self.prev_landmark)

        self.prev_landmark = overall_pose
        return covariance.flatten().tolist()

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = LandmarkConverter()
    node.run()
