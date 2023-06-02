import rospy
from copy import deepcopy
import numpy as np
from typing import Optional, List
from nav_msgs.msg import Odometry
from tf_conversions import transformations
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise

from .helpers import (
    landmark_to_measurement,
    odometry_to_measurement,
    state_transition_fn,
    sqrt_func,
    NUM_STATES,
    NUM_STATES_1ST_ORDER,
    NUM_MEASUREMENTS,
    LANDMARK_COVARIANCE_INDICES,
)


class TagFastForward:
    def __init__(self, dt: float, play_forward_buffer_size: int) -> None:
        self.dt = dt
        self.play_forward_buffer_size = play_forward_buffer_size

        z_std = 0.001
        measurement_var = 0.001**2

        points = MerweScaledSigmaPoints(
            NUM_STATES, alpha=0.3, beta=2.0, kappa=0.1, sqrt_method=sqrt_func
        )
        self.filter = UnscentedKalmanFilter(
            dim_x=NUM_STATES,
            dim_z=NUM_MEASUREMENTS,
            dt=dt,
            hx=None,
            fx=state_transition_fn,
            points=points,
            sqrt_fn=sqrt_func,
        )

        # state uncertainty
        self.filter.R = np.eye(NUM_MEASUREMENTS) * z_std**2

        # process uncertainty
        self.filter.Q = Q_discrete_white_noise(
            dim=NUM_STATES_1ST_ORDER, dt=self.dt, var=measurement_var, block_size=2
        )

        self.current_index = 0
        self.odom_messages: List[Odometry] = []

    def get_rpy(self, quaternion: Quaternion):
        return transformations.euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )

    def fast_forward(self, msg: PoseWithCovarianceStamped) -> PoseWithCovarianceStamped:
        start_time = msg.header.stamp.to_sec()
        now = rospy.Time.now().to_sec()
        lag = now - start_time
        num_samples = round(lag / self.dt)
        if num_samples > 0:
            self.reset_filter_to_landmark(msg)
            self.current_index = 0
            for forwarded_time in np.linspace(start_time, now, num_samples):
                odom_msg = self.find_nearest_odom(forwarded_time)
                if odom_msg is not None:
                    self.update_odometry(odom_msg)
                self.filter.predict()
            self.odom_messages = []

            roll, pitch, _ = self.get_rpy(msg.pose.pose.orientation)
            result = deepcopy(msg)
            result.pose.pose.position.x = self.filter.x[0]
            result.pose.pose.position.y = self.filter.x[1]
            result.pose.pose.orientation = Quaternion(
                *transformations.quaternion_from_euler(roll, pitch, self.filter.x[2])
            )
            covariance = list(result.pose.covariance)
            for mat_index, msg_index in LANDMARK_COVARIANCE_INDICES.items():
                covariance[msg_index] = self.filter.P[mat_index]
            result.pose.covariance = covariance
            return result
        else:
            return msg

    def find_nearest_odom(self, timestamp: float) -> Optional[Odometry]:
        selected_msg = None
        index = self.current_index
        for index in range(self.current_index, len(self.odom_messages)):
            msg = self.odom_messages[index]
            odom_timestamp = msg.header.stamp.to_sec()
            time_error = odom_timestamp - timestamp
            if abs(time_error) < self.dt:
                selected_msg = msg
                break
            if time_error > 0.0:
                break
        self.current_index = index
        return selected_msg

    def reset_filter_to_landmark(self, msg: PoseWithCovarianceStamped) -> None:
        measurement, measurement_noise = landmark_to_measurement(msg)
        self.filter.x = np.zeros(NUM_STATES)
        self.filter.x[0:NUM_STATES_1ST_ORDER] = measurement
        self.filter.P = np.eye(NUM_STATES)
        self.filter.P[
            0:NUM_STATES_1ST_ORDER, 0:NUM_STATES_1ST_ORDER
        ] = measurement_noise

    def odometry_measurement_fn(self, state):
        return state[NUM_STATES_1ST_ORDER:NUM_STATES]

    def update_odometry(self, msg: Odometry) -> None:
        measurement, measurement_noise = odometry_to_measurement(msg)
        self.filter.update(
            measurement, R=measurement_noise, hx=self.odometry_measurement_fn
        )

    def record_odometry(self, msg: Odometry) -> None:
        self.odom_messages.append(msg)
