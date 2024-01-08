import numpy as np
import tf.transformations
from geometry_msgs.msg import Pose, Quaternion, Vector3


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


def main():
    tag_matrix = pose_to_matrix(
        Pose(position=Vector3(x=1.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    )
    bundle_matrix = pose_to_matrix(
        Pose(position=Vector3(x=1.0, y=1.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
    )

    robot_matrix = bundle_matrix @ np.linalg.inv(tag_matrix)
    robot_pose = matrix_to_pose(robot_matrix)
    print(robot_pose)  # should be x=2.0, y=1.0, z=0.0


if __name__ == "__main__":
    main()
