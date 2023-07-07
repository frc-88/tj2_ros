import scipy
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
def normalize_theta(theta):
    # normalize theta to -pi..pi
    theta = theta % (2 * math.pi)
    if abs(theta) > math.pi:
        if theta > 0:
            return theta - 2 * math.pi
        else:
            return theta + 2 * math.pi
    return theta


@njit
def state_transition_fn(state, dt):
    x = state[0]
    y = state[1]
    theta = normalize_theta(state[2])

    vx_prev = state[3]
    vy_prev = state[4]

    vx = vx_prev * math.cos(theta) - vy_prev * math.sin(theta)
    vy = vx_prev * math.sin(theta) + vy_prev * math.cos(theta)
    vt = state[5]

    next_state = np.copy(state)
    next_state[0] = x + dt * vx
    next_state[1] = y + dt * vy
    next_state[2] = theta + dt * vt

    return next_state


def sqrt_func(x):
    try:
        result = scipy.linalg.cholesky(x)
    except scipy.linalg.LinAlgError:
        x = (x + x.T) / 2
        result = scipy.linalg.cholesky(x)
    return result


@njit
def jit_update(x, P, H, z, R):
    y = z - H @ x  # error (residual)
    PHT = P @ H.T
    S = H @ PHT + R  # project system uncertainty into measurement space
    SI = np.linalg.inv(S)
    K = PHT @ SI  # map system uncertainty into kalman gain
    x = x + K @ y  # predict new x with residual scaled by the kalman gain
    P = (np.eye(len(x)) - K @ H) @ P  # updated state covariance matrix
    return x, P


@njit
def jit_predict(x, P, Q, dt):
    n = len(x)
    n_half = n // 2
    F = np.eye(n)
    F[0:n_half, n_half:n] = np.eye(n_half) * dt
    x = state_transition_fn(x, dt)
    P = F @ P @ F.T + Q
    return x, P
