import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from tj2_tools.robot_state import Pose2d, Velocity
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise


class FilterModel:
    def __init__(
        self,
        dt: float,
    ) -> None:
        self.num_states = 6
        self.num_measurements = 3

        def fx(state, dt):
            # # state transition function - predict next state based
            state_out = np.zeros_like(state)
            state_out[0:3] = state[3:6] * dt + state[0:3]
            state_out[3:6] = state[3:6]
            return state_out

        def hx(state):
            # measurement function - convert state into a measurement
            return state[0:3]

        points = MerweScaledSigmaPoints(self.num_states, alpha=0.1, beta=2.0, kappa=-1)
        self.filter = UnscentedKalmanFilter(
            dim_x=self.num_states,
            dim_z=self.num_measurements,
            dt=dt,
            fx=fx,
            hx=hx,
            points=points,
        )
        self.filter.x = np.zeros(self.num_states)  # initial state
        self.filter.P = 0.2  # initial uncertainty
        z_std = 0.1
        self.filter.R = np.diag([z_std**2, z_std**2, z_std**2])
        self.filter.Q = Q_discrete_white_noise(
            dim=self.num_measurements, dt=dt, var=0.01**2, block_size=2
        )

        self.prev_odom_time = 0.0

    def predict(self) -> None:
        self.filter.predict()

    def update_landmark(self, msg: PoseWithCovarianceStamped) -> None:
        pose = Pose2d.from_ros_pose(msg.pose.pose)
        covariance = np.array(msg.pose.covariance).reshape((6, 6))
        measurement_noise = np.eye(self.num_measurements)
        measurement_noise[0, 0] = covariance[0, 0]
        measurement_noise[1, 1] = covariance[1, 1]
        measurement_noise[2, 2] = covariance[5, 5]
        self.filter.update(np.array(pose.to_list()), R=measurement_noise)

    def update_odometry(self, msg: Odometry) -> None:
        if self.prev_odom_time == 0.0:
            self.prev_odom_time = msg.header.stamp.to_sec()
            return
        delta_time = msg.header.stamp.to_sec() - self.prev_odom_time

        velocity = Velocity.from_ros_twist(msg.twist.twist)
        pose = Pose2d.from_ros_pose(msg.pose.pose)
        pose = pose.project(velocity, delta_time)

        covariance = np.array(msg.pose.covariance).reshape((6, 6))
        measurement_noise = np.eye(self.num_measurements)
        measurement_noise[0, 0] = covariance[0, 0]
        measurement_noise[1, 1] = covariance[1, 1]
        measurement_noise[2, 2] = covariance[5, 5]
        self.filter.update(
            np.array(pose.to_list()),
            R=measurement_noise,
        )
