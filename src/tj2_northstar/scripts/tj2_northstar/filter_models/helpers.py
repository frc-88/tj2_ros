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


def nearestPD(A):
    """Find the nearest positive-definite matrix to input

    A Python/Numpy port of John D'Errico's `nearestSPD` MATLAB code [1], which
    credits [2].

    [1] https://www.mathworks.com/matlabcentral/fileexchange/42885-nearestspd

    [2] N.J. Higham, "Computing a nearest symmetric positive semidefinite
    matrix" (1988): https://doi.org/10.1016/0024-3795(88)90223-6
    """

    B = (A + A.T) / 2
    _, s, V = np.linalg.svd(B)

    H = np.dot(V.T, np.dot(np.diag(s), V))

    A2 = (B + H) / 2

    A3 = (A2 + A2.T) / 2

    if isPD(A3):
        return A3

    spacing = np.spacing(np.linalg.norm(A))
    # The above is different from [1]. It appears that MATLAB's `chol` Cholesky
    # decomposition will accept matrixes with exactly 0-eigenvalue, whereas
    # Numpy's will not. So where [1] uses `eps(mineig)` (where `eps` is Matlab
    # for `np.spacing`), we use the above definition. CAVEAT: our `spacing`
    # will be much larger than [1]'s `eps(mineig)`, since `mineig` is usually on
    # the order of 1e-16, and `eps(1e-16)` is on the order of 1e-34, whereas
    # `spacing` will, for Gaussian random matrixes of small dimension, be on
    # othe order of 1e-16. In practice, both ways converge, as the unit test
    # below suggests.
    I = np.eye(A.shape[0])
    k = 1
    while not isPD(A3):
        mineig = np.min(np.real(np.linalg.eigvals(A3)))
        A3 += I * (-mineig * k**2 + spacing)
        k += 1

    return A3


def isPD(B):
    """Returns true when input is positive-definite, via Cholesky"""
    try:
        _ = np.linalg.cholesky(B)
        return True
    except np.linalg.LinAlgError:
        return False


def sqrt_func(x):
    try:
        result = scipy.linalg.cholesky(x)
    except scipy.linalg.LinAlgError:
        x = (x + x.T) / 2
        result = scipy.linalg.cholesky(x)
    return result
