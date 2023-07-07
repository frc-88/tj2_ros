import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
from tj2_tools.robot_state import Pose2d


def amcl_and_landmark_agree(
    amcl_pose: PoseWithCovarianceStamped,
    landmark: PoseWithCovarianceStamped,
    roll_pitch_threshold: float,
    ground_distance_threshold: float,
    ground_angle_threshold: float,
) -> bool:
    landmark_quat = landmark.pose.pose.orientation
    roll, pitch, _ = tf.transformations.euler_from_quaternion(
        (landmark_quat.x, landmark_quat.y, landmark_quat.z, landmark_quat.w)
    )
    if abs(roll) > roll_pitch_threshold or abs(pitch) > roll_pitch_threshold:
        # check if the landmark is a reasonable roll and pitch
        return False

    landmark_pose2d = Pose2d.from_ros_pose(landmark.pose.pose)
    amcl_pose2d = Pose2d.from_ros_pose(amcl_pose.pose.pose)

    distance_delta = landmark_pose2d.distance(amcl_pose2d)
    angle_delta = abs(landmark_pose2d.theta - amcl_pose2d.theta)

    return (
        distance_delta < ground_distance_threshold
        and angle_delta < ground_angle_threshold
    )
