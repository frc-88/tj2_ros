import numpy as np
from typing import Tuple
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from tj2_tools.robot_state import Pose2d, Velocity
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


class FilterModel:
    def __init__(
        self,
        dt: float,
    ) -> None:
        self.dt = dt
        self.num_states = 6
        self.num_states_1st_order = self.num_states // 2
        self.num_measurements = 3

        z_std = 0.001
        measurement_var = 0.001**2

        n = self.num_states
        m = self.num_measurements
        n_half = self.num_states_1st_order

        self.filter = KalmanFilter(dim_x=n, dim_z=m)
        self.filter.x = np.zeros(n)  # initial state

        # initial uncertainty
        self.filter.P = np.eye(n) * 0.2

        # x' + vx * dt = x (vx is in the same frame as x). Same applies for y and theta
        self.filter.F = np.eye(n)
        self.filter.F[0:n_half, n_half:n] = np.eye(n_half) * self.dt

        # state uncertainty
        self.filter.R = np.eye(n) * z_std**2

        # process uncertainty
        self.filter.Q = Q_discrete_white_noise(
            dim=n_half, dt=self.dt, var=measurement_var, block_size=2
        )

        # measurement function for landmarks. Use only position.
        self.landmark_H = np.zeros((m, n))
        self.landmark_H[0:n_half, 0:n_half] = np.eye(m)

        # measurement function for odometry. Use only velocity.
        self.odom_H = np.zeros((m, n))
        self.odom_H[0:n_half, n_half:n] = np.eye(m)

        self.landmark_covariance_indices = {
            (0, 0): self.get_index(0, 0, n),
            (1, 1): self.get_index(1, 1, n),
            (2, 2): self.get_index(5, 5, n),
        }  # fmt: off
        self.odom_covariance_indices = {
            (0, 0): self.get_index(0, 0, n),
            (1, 1): self.get_index(1, 1, n),
            (2, 2): self.get_index(5, 5, n),
        }  # fmt: off

    @staticmethod
    def get_index(row: int, col: int, row_length: int) -> int:
        return row * row_length + col

    def predict(self) -> None:
        self.filter.predict()

    def update_landmark(self, msg: PoseWithCovarianceStamped) -> None:
        pose = Pose2d.from_ros_pose(msg.pose.pose)
        measurement_noise = np.eye(self.num_measurements)
        for mat_index, msg_index in self.landmark_covariance_indices.items():
            measurement_noise[mat_index] = msg.pose.covariance[msg_index]
        self.filter.update(
            np.array(pose.to_list()), R=measurement_noise, H=self.landmark_H
        )

    def update_odometry(self, msg: Odometry) -> None:
        relative_velocity = Velocity.from_ros_twist(msg.twist.twist)
        prev_state = self.get_pose()
        global_velocity = relative_velocity.rotate_by(prev_state.theta)
        # extract only x, y from rotated velocity since theta is in global already
        measurement = np.array(
            [global_velocity.x, global_velocity.y, relative_velocity.theta]
        )
        print(global_velocity, relative_velocity)

        measurement_noise = np.eye(self.num_measurements)
        for mat_index, msg_index in self.odom_covariance_indices.items():
            measurement_noise[mat_index] = msg.twist.covariance[msg_index]

        self.filter.update(
            measurement,
            R=measurement_noise,
            H=self.odom_H,
        )

    def get_pose(self) -> Pose2d:
        return Pose2d(x=self.filter.x[0], y=self.filter.x[1], theta=self.filter.x[2])

    def get_velocity(self) -> Velocity:
        return Velocity(x=self.filter.x[3], y=self.filter.x[4], theta=self.filter.x[5])

    def get_state(self) -> Tuple[Pose2d, Velocity]:
        return (self.get_pose(), self.get_velocity())

    def get_covariance(self) -> np.ndarray:
        return self.filter.P
