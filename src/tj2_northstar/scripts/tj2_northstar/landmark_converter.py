#!/usr/bin/env python3
from typing import List, Optional
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from tj2_tools.transforms import lookup_transform


class LandmarkConverter:
    def __init__(self) -> None:
        rospy.init_node("landmark_converter")

        self.base_frame = str(rospy.get_param("~base_frame", "base_tilt_link"))
        self.field_frame = str(rospy.get_param("~field_frame", "field"))
        self.landmark_ids = frozenset(rospy.get_param("~landmark_ids", [0]))
        base_covariance = rospy.get_param(
            "~covariance", np.eye(6, dtype=np.float64).flatten().tolist()
        )
        assert (
            len(base_covariance) == 36
        ), f"Invalid covariance. Length is {len(base_covariance)}: {base_covariance}"
        self.base_covariance = np.array(base_covariance).reshape((6, 6))

        self.landmark = PoseWithCovarianceStamped()
        self.landmark.header.frame_id = self.base_frame

        self.buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.buffer)

        self.tag_sub = rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray, self.tags_callback, queue_size=10
        )
        self.landmark_pub = rospy.Publisher(
            "landmark", PoseWithCovarianceStamped, queue_size=10
        )

    def tags_callback(self, msg: AprilTagDetectionArray) -> None:
        assert msg.detections is not None
        landmark_pose: Optional[PoseStamped] = None
        individual_measurements = []
        for detection in msg.detections:
            ids = frozenset(detection.id)
            detection_pose = PoseStamped()
            detection_pose.header = detection.pose.header
            detection_pose.pose = detection.pose.pose.pose
            if self.landmark_ids == ids:
                landmark_pose = detection_pose
            elif len(ids) == 1:
                individual_measurements.append(detection_pose)
        if landmark_pose is not None:
            self.publish_landmark_from_tf()
            # self.publish_landmark(landmark_pose)
        if len(individual_measurements) > 0:
            self.update_covariance(individual_measurements)

    def publish_landmark_from_tf(self) -> None:
        transform = lookup_transform(self.buffer, self.field_frame, self.base_frame)
        zero_pose = PoseStamped()
        zero_pose.header.frame_id = self.base_frame
        zero_pose.pose.orientation.w = 1.0
        base_in_field = tf2_geometry_msgs.do_transform_pose(zero_pose, transform)
        self.landmark.header = base_in_field.header
        self.landmark.pose.pose = base_in_field.pose
        self.landmark_pub.publish(self.landmark)

    def publish_inverted_landmark(self, pose: PoseStamped) -> None:
        inverted_pose = self.invert_transform_pose(pose)
        if inverted_pose is None:
            return
        self.landmark.header = inverted_pose.header
        self.landmark.pose.pose = inverted_pose.pose
        self.landmark_pub.publish(self.landmark)

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

        inverse_mat = tf.transformations.inverse_matrix(
            tf.transformations.translation_matrix(translation)
        )

        inverse_mat = tf.transformations.concatenate_matrices(
            tf.transformations.inverse_matrix(
                tf.transformations.quaternion_matrix(rotation)
            ),
            inverse_mat,
        )

        forward_translation = tf.transformations.translation_from_matrix(inverse_mat)
        forward_rotation = tf.transformations.quaternion_from_matrix(inverse_mat)
        inverse_pose = PoseStamped()
        # frame is unassigned as the computed frame doesn't exist yet
        inverse_pose.header.frame_id = ""
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
        # 1 -> 1
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

    def update_covariance(self, poses: List[PoseStamped]) -> None:
        distances = [self.get_pose_distance(pose) for pose in poses]
        aggregate_distance = float(np.median(distances))

        covariance = self.base_covariance * self.num_tags_covariance_scale(len(poses))
        covariance *= self.pose_distance_covariance_scale(aggregate_distance)
        self.landmark.pose.covariance = covariance.flatten().tolist()

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = LandmarkConverter()
    node.run()
