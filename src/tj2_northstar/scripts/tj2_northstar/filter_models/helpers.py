import math
from numba import njit
import numpy as np
from typing import Tuple, Dict
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from tj2_tools.robot_state import Pose2d, Velocity

NUM_MEASUREMENTS = 3
NUM_POSE_STATES_ROS = 6
NUM_STATES = 6
NUM_STATES_1ST_ORDER = 3


def get_index(row: int, col: int, row_length: int) -> int:
    return row * row_length + col


LANDMARK_COVARIANCE_INDICES: Dict[Tuple[int, int], int] = {
    (0, 0): get_index(0, 0, NUM_POSE_STATES_ROS),
    (1, 1): get_index(1, 1, NUM_POSE_STATES_ROS),
    (2, 2): get_index(5, 5, NUM_POSE_STATES_ROS),
}  # fmt: off
ODOM_COVARIANCE_INDICES: Dict[Tuple[int, int], int] = {
    (0, 0): get_index(0, 0, NUM_POSE_STATES_ROS),
    (1, 1): get_index(1, 1, NUM_POSE_STATES_ROS),
    (2, 2): get_index(5, 5, NUM_POSE_STATES_ROS),
}  # fmt: off


def landmark_to_measurement(
    msg: PoseWithCovarianceStamped,
) -> Tuple[np.ndarray, np.ndarray]:
    pose = Pose2d.from_ros_pose(msg.pose.pose)
    measurement = np.array(pose.to_list())

    measurement_noise = np.eye(NUM_MEASUREMENTS)
    for mat_index, msg_index in LANDMARK_COVARIANCE_INDICES.items():
        measurement_noise[mat_index] = msg.pose.covariance[msg_index]

    return measurement, measurement_noise


def odometry_to_measurement(msg: Odometry) -> Tuple[np.ndarray, np.ndarray]:
    velocity = Velocity.from_ros_twist(msg.twist.twist)
    measurement = np.array(velocity.to_list())

    measurement_noise = np.eye(NUM_MEASUREMENTS)
    for mat_index, msg_index in ODOM_COVARIANCE_INDICES.items():
        measurement_noise[mat_index] = msg.twist.covariance[msg_index]

    return measurement, measurement_noise


@njit
def state_transition_fn(state, dt):
    x = state[0]
    y = state[1]
    theta = state[2]

    vx = x * math.cos(theta) - y * math.sin(theta)
    vy = x * math.sin(theta) + y * math.cos(theta)
    vt = state[5]

    next_state = state
    next_state[0] = x + dt * vx
    next_state[1] = y + dt * vy
    next_state[2] = theta + dt * vt

    return next_state
