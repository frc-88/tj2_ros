#!/usr/bin/env python3
from dataclasses import dataclass
from functools import cached_property

import numpy as np
import rospy
import tf.transformations
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion, Vector3
from std_msgs.msg import Header


def pose_to_matrix(pose: Pose) -> np.ndarray:
    matrix = tf.transformations.quaternion_matrix(
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    )
    matrix[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return matrix


def matrix_to_pose(matrix: np.ndarray) -> Pose:
    position = tf.transformations.translation_from_matrix(matrix)
    orientation = tf.transformations.quaternion_from_matrix(matrix)
    return Pose(
        position=Vector3(x=position[0], y=position[1], z=position[2]),
        orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]),
    )


@dataclass
class BundleConfig:
    id: int
    size: float
    frame_id: str
    pose: Pose

    @cached_property
    def transform(self) -> np.ndarray:
        return pose_to_matrix(self.pose)


class CudaLandmarkConverter:
    def __init__(self) -> None:
        rospy.init_node("cuda_landmark_converter")

        self.base_frame = str(rospy.get_param("~base_frame", "base_tilt_link"))
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.field_frame = str(rospy.get_param("~field_frame", "field"))
        self.max_tag_distance = float(rospy.get_param("~max_tag_distance", 2.0))
        base_covariance = rospy.get_param("~covariance", np.eye(6, dtype=np.float64).flatten().tolist())
        assert len(base_covariance) == 36, f"Invalid covariance. Length is {len(base_covariance)}: {base_covariance}"
        self.base_covariance = np.array(base_covariance).reshape((6, 6))
        bundles_config = rospy.get_param("~tag_bundles", [])
        assert isinstance(bundles_config, list) and len(bundles_config) > 0, "No tag bundles specified!"
        bundle_config = []
        for bundle in bundles_config:
            if bundle["name"] == self.field_frame:
                bundle_config = bundle
                break
        if len(bundle_config) == 0:
            raise ValueError(f"Bundle {self.field_frame} not found!")
        self.bundle = {
            bundle["id"]: BundleConfig(
                id=bundle["id"],
                size=bundle["size"],
                frame_id=self.map_frame,
                pose=Pose(
                    position=Vector3(x=bundle["x"], y=bundle["y"], z=bundle["z"]),
                    orientation=Quaternion(
                        x=bundle["qx"],
                        y=bundle["qy"],
                        z=bundle["qz"],
                        w=bundle["qw"],
                    ),
                ),
            )
            for bundle in bundle_config
        }

        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tags_callback, queue_size=10)
        self.landmark_pub = rospy.Publisher("landmark", PoseWithCovarianceStamped, queue_size=10)

    def pose_distance_covariance_scale(self, distance: float) -> float:
        if distance < 0.5:
            return 10.0
        return 0.25 * distance**2.0

    def tags_callback(self, msg: AprilTagDetectionArray) -> None:
        assert msg.detections is not None

        for detection in msg.detections:
            detection: AprilTagDetection
            if len(detection.id) != 1:
                continue
            tag_matrix = pose_to_matrix(detection.pose.pose.pose)
            tag_distance = float(np.linalg.norm(tag_matrix[:3, 3]))
            bundle = self.bundle[detection.id[0]]
            bundle_matrix = bundle.transform
            robot_matrix = bundle_matrix @ np.linalg.inv(tag_matrix)
            robot_pose = matrix_to_pose(robot_matrix)
            covariance = self.base_covariance * self.pose_distance_covariance_scale(tag_distance)

            landmark = PoseWithCovarianceStamped(
                header=Header(frame_id=self.map_frame),
                pose=PoseWithCovariance(
                    pose=robot_pose,
                    covariance=covariance.flatten().tolist(),
                ),
            )
            self.landmark_pub.publish(landmark)

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = CudaLandmarkConverter()
    node.run()
