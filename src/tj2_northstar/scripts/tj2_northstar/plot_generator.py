#!/usr/bin/env python3
import rospy
import numpy as np
from typing import List
import matplotlib
from matplotlib.axis import Axis
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tj2_tools.robot_state import Pose2d


def odometries_to_array(msgs: List[Odometry]) -> np.ndarray:
    data = []
    for msg in msgs:
        timestamp = msg.header.stamp.to_sec()
        pose = Pose2d.from_ros_pose(msg.pose.pose)
        data.append([timestamp] + pose.to_list())
    return np.array(data)


class OdometryLine:
    def __init__(self, axis: Axis, prefix: str, enabled="xyt") -> None:
        self.states = []
        self.enabled = enabled
        self.x_line = (
            axis.plot([], [], label=f"{prefix}-x")[0] if self.is_x_enabled() else None
        )
        self.y_line = (
            axis.plot([], [], label=f"{prefix}-y")[0] if self.is_y_enabled() else None
        )
        self.theta_line = (
            axis.plot([], [], label=f"{prefix}-theta")[0]
            if self.is_theta_enabled()
            else None
        )
        self.axis = axis

    def is_x_enabled(self):
        return "x" in self.enabled

    def is_y_enabled(self):
        return "y" in self.enabled

    def is_theta_enabled(self):
        return "theta" in self.enabled

    def is_any_enabled(self):
        return len(self.enabled) > 0

    def update(self, msg: Odometry):
        if not self.is_any_enabled():
            return
        self.states.append(msg)
        data = odometries_to_array(self.states)
        x_data = data[:, 0]
        self.axis.set_xlim(min(x_data), max(x_data))
        flattened = data[:, 1:].flatten()
        min_y = min(flattened)
        max_y = max(flattened)
        self.axis.set_ylim(min_y, max_y)
        if self.is_x_enabled():
            self.x_line.set_xdata(x_data)
            self.x_line.set_ydata(data[:, 1])
        if self.is_y_enabled():
            self.y_line.set_xdata(x_data)
            self.y_line.set_ydata(data[:, 2])
        if self.is_theta_enabled():
            self.theta_line.set_xdata(x_data)
            self.theta_line.set_ydata(data[:, 3])


class ScalarLine:
    def __init__(self, axis: Axis, label: str, enabled: bool = True) -> None:
        self.enabled = enabled
        self.times = []
        self.states = []
        self.line = axis.plot([], [], label=label)[0]
        self.axis = axis

    def update(self, timestamp: float, value: float):
        if not self.enabled:
            return
        self.times.append(timestamp)
        self.states.append(value)
        self.line.set_xdata(self.times)
        self.line.set_ydata(self.states)
        self.axis.set_xlim(min(self.times), max(self.times))
        self.axis.set_ylim(min(self.states), max(self.states))


class Plotter:
    def __init__(self) -> None:
        rospy.init_node("plot_generator")

        self.plot_delay = 0.01

        self.init()
        self.forwarded_dist_line = ScalarLine(self.error_over_time_plot, "distance")
        self.forwarded_angle_line = ScalarLine(self.error_over_time_plot, "angle")
        self.filter_line = OdometryLine(
            self.state_over_time_plot, "filter", enabled="xy"
        )
        self.odom_line = OdometryLine(self.state_over_time_plot, "odom", enabled="")

        self.prev_landmark = None

        self.filter_state_sub = rospy.Subscriber(
            "filter_state",
            Odometry,
            self.filter_state_callback,
            queue_size=10,
        )
        self.odom_sub = rospy.Subscriber(
            "odom",
            Odometry,
            self.odom_callback,
            queue_size=10,
        )
        self.landmark_sub = rospy.Subscriber(
            "landmark", PoseWithCovarianceStamped, self.landmark_callback, queue_size=10
        )
        self.forwarded_landmark_sub = rospy.Subscriber(
            "landmark/forwarded",
            PoseWithCovarianceStamped,
            self.forwarded_landmark_callback,
            queue_size=10,
        )

    def init(self):
        self.fig = plt.figure()

        plt.tight_layout()
        plt.ion()
        self.fig.show()

        self.error_over_time_plot = self.fig.add_subplot(1, 2, 1)
        self.state_over_time_plot = self.fig.add_subplot(1, 2, 2)

        plt.tight_layout()
        plt.ion()
        plt.legend()
        self.fig.show()

    def filter_state_callback(self, msg: Odometry) -> None:
        self.filter_line.update(msg)

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_line.update(msg)

    def landmark_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.prev_landmark = msg

    def forwarded_landmark_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if self.prev_landmark is None:
            return
        forwarded = Pose2d.from_ros_pose(msg.pose.pose)
        landmark = Pose2d.from_ros_pose(self.prev_landmark.pose.pose)
        distance = forwarded.distance(landmark)
        angle = forwarded.theta - landmark.theta
        self.forwarded_dist_line.update(msg.header.stamp.to_sec(), distance)
        self.forwarded_angle_line.update(msg.header.stamp.to_sec(), angle)

    def pause(self) -> None:
        backend = plt.rcParams["backend"]
        if backend in matplotlib.rcsetup.interactive_bk:
            figManager = matplotlib._pylab_helpers.Gcf.get_active()
            if figManager is not None:
                canvas = figManager.canvas
                if canvas.figure.stale:
                    canvas.draw()
                canvas.start_event_loop(self.plot_delay)
                return

    def run(self) -> None:
        while not rospy.is_shutdown():
            self.pause()
            rospy.sleep(self.plot_delay)


def main():
    plotter = Plotter()
    plotter.run()


if __name__ == "__main__":
    main()
