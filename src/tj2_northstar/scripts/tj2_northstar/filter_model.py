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
        self.num_measurements = 3

        self.filter = KalmanFilter(dim_x=self.num_states, dim_z=self.num_measurements)
        self.filter.x = np.zeros(self.num_states)  # initial state

        # initial uncertainty
        self.filter.P = np.eye(self.num_states) * 0.2

        # x' + vx * dt = x (vx is in the same frame as x). Same applies for y and theta
        self.filter.F = np.eye(self.num_states)
        self.filter.F[0:3, 3:6] = np.eye(3) * self.dt

        # state uncertainty
        z_std = 0.1
        self.filter.R = np.eye(self.num_states) * z_std**2

        # process uncertainty
        self.filter.Q = Q_discrete_white_noise(
            dim=self.num_measurements, dt=self.dt, var=0.01**2, block_size=2
        )

        # measurement function for landmarks. Use only position.
        self.landmark_H = np.zeros((self.num_states, self.num_measurements))
        self.landmark_H[0:3, 0:3] = np.eye(self.num_measurements)

        # measurement function for odometry. Use only velocity.
        self.odom_H = np.zeros((self.num_states, self.num_measurements))
        self.odom_H[3:6, 0:3] = np.eye(self.num_measurements)

        self.landmark_covariance_indices = {
            (0, 0): 0 * 0,
            (1, 1): 1 * 1,
            (2, 2): 5 * 5,
        }  # fmt: off
        self.odom_covariance_indices = {
            (0, 0): 0 * 0,
            (1, 1): 1 * 1,
            (2, 2): 5 * 5,
        }  # fmt: off

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
        measurement = np.array([global_velocity.x, global_velocity.y, prev_state.theta])

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
