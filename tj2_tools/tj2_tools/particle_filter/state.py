import numpy as np
import tf_conversions
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2D


class State:
    def __init__(self):
        self.type = ""
        self.stamp = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.t = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vt = 0.0

    @classmethod
    def from_state(cls, other):
        if not isinstance(other, cls):
            raise ValueError("%s is not of type %s" % (repr(other), cls))
        self = cls()
        self.type = other.type
        self.stamp = other.stamp
        self.x = other.x
        self.y = other.y
        self.z = other.z
        self.t = other.t
        self.vx = other.vx
        self.vy = other.vy
        self.vz = other.vz
        self.vt = other.vt
        return self

    @classmethod
    def from_odom(cls, msg):
        if not isinstance(msg, Odometry):
            raise ValueError("%s is not of type %s" % (repr(msg), Odometry))
        self = cls()
        self.type = "odom"
        self.stamp = msg.header.stamp.to_sec()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.t = self.quat_to_yaw(msg.pose.pose.orientation)
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        self.vt = msg.twist.twist.angular.z
        return self

    @classmethod
    def from_detect(cls, msg):
        if not isinstance(msg, Detection2D):
            raise ValueError("%s is not of type %s" % (repr(msg), Detection2D))
        self = cls()
        self.type = msg.results[0].id
        self.stamp = msg.header.stamp.to_sec()

        pose = msg.results[0].pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z
        self.t = self.quat_to_yaw(pose.orientation)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vt = 0.0
        return self

    @staticmethod
    def quat_to_yaw(quaternion):
        return tf_conversions.transformations.euler_from_quaternion((
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        ))[2]

    def __str__(self):
        return f"<{self.type}>(" \
               f"x={self.x:0.4f}, y={self.y:0.4f}, z={self.z:0.4f}, t={self.t:0.4f}, " \
               f"vx={self.vx:0.4f}, vy={self.vy:0.4f}, vz={self.vz:0.4f}, vt={self.vt:0.4f}) @ {self.stamp}"


class DeltaTimer:
    def __init__(self):
        self.prev_stamp = None

    def dt(self, timestamp):
        if self.prev_stamp is None:
            self.prev_stamp = timestamp
        dt = timestamp - self.prev_stamp
        self.prev_stamp = timestamp
        return dt


class VelocityFilter:
    def __init__(self, k):
        self.k = k
        self.prev_value = None
        self.speed = 0.0

    def update(self, dt, value):
        if self.prev_value is None:
            self.prev_value = value
        raw_delta = (value - self.prev_value) / dt
        self.prev_value = value
        if self.k is None:
            self.speed = raw_delta
        else:
            self.speed = self.k * (raw_delta - self.speed)
        return self.speed


class DeltaMeasurement:
    def __init__(self):
        smooth_k = None
        self.vx_filter = VelocityFilter(smooth_k)
        self.vy_filter = VelocityFilter(smooth_k)
        self.vz_filter = VelocityFilter(smooth_k)
        self.timer = DeltaTimer()

    def update(self, state: State):
        dt = self.timer.dt(state.stamp)
        if dt == 0.0:
            return state
        new_state = State.from_state(state)
        new_state.vx = self.vx_filter.update(dt, state.x)
        new_state.vy = self.vy_filter.update(dt, state.y)
        new_state.vz = self.vz_filter.update(dt, state.z)
        return new_state


class InputVector:
    def __init__(self):
        self.odom_timer = DeltaTimer()
        self.meas = DeltaMeasurement()
        self.meas_input = np.array([0.0, 0.0, 0.0, 0.0])
        self.vector = np.array([0.0, 0.0, 0.0, 0.0])

    def odom_update(self, odom_state: State):
        dt = self.odom_timer.dt(odom_state.stamp)
        self.vector = np.array([odom_state.vx, odom_state.vy, odom_state.vz, odom_state.vt])
        self.vector += self.meas_input
        return dt

    def meas_update(self, meas_state: State):
        new_state = self.meas.update(meas_state)
        self.meas_input = np.array([new_state.vx, new_state.vy, new_state.vz, new_state.vt])
        return new_state

    def get_vector(self):
        return self.vector

