import numpy as np
from typing import Tuple
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise

from tj2_tools.robot_state import Pose2d, Velocity

from .filter_model import FilterModel
from .helpers import (
    landmark_to_measurement,
    odometry_to_measurement,
    state_transition_fn,
    NUM_STATES,
    NUM_STATES_1ST_ORDER,
    NUM_MEASUREMENTS,
)


class DriveUnscentedKalmanFilterModel(FilterModel):
    def __init__(self, dt: float) -> None:
        self.dt = dt

        z_std = 0.001
        measurement_var = 0.001**2

        points = MerweScaledSigmaPoints(4, alpha=0.1, beta=2.0, kappa=-1)
        self.filter = UnscentedKalmanFilter(
            dim_x=NUM_STATES,
            dim_z=NUM_MEASUREMENTS,
            dt=dt,
            hx=None,
            fx=state_transition_fn,
            points=points,
        )

        # state uncertainty
        self.filter.R = np.eye(NUM_STATES) * z_std**2

        # process uncertainty
        self.filter.Q = Q_discrete_white_noise(
            dim=NUM_STATES_1ST_ORDER, dt=self.dt, var=measurement_var, block_size=2
        )

    def predict(self) -> None:
        self.filter.predict()

    def landmark_measurement_fn(self, state):
        return state[0:NUM_STATES_1ST_ORDER]

    def update_landmark(self, msg: PoseWithCovarianceStamped) -> None:
        measurement, measurement_noise = landmark_to_measurement(msg)
        self.filter.update(
            measurement, R=measurement_noise, hx=self.landmark_measurement_fn
        )

    def odometry_measurement_fn(self, state):
        return state[NUM_STATES_1ST_ORDER:NUM_STATES]

    def update_odometry(self, msg: Odometry) -> None:
        measurement, measurement_noise = odometry_to_measurement(msg)
        self.filter.update(
            measurement, R=measurement_noise, hx=self.odometry_measurement_fn
        )

    def get_pose(self) -> Pose2d:
        return Pose2d(x=self.filter.x[0], y=self.filter.x[1], theta=self.filter.x[2])

    def get_velocity(self) -> Velocity:
        return Velocity(x=self.filter.x[3], y=self.filter.x[4], theta=self.filter.x[5])

    def get_state(self) -> Tuple[Pose2d, Velocity]:
        return (self.get_pose(), self.get_velocity())

    def get_covariance(self) -> np.ndarray:
        return self.filter.P
