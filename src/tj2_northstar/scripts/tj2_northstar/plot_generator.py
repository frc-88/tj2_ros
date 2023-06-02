#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib
from typing import List
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
    def __init__(self, axis, prefix: str) -> None:
        self.states = []
        self.x_line = axis.plot([], [], label=f"{prefix}-x")[0]
        self.y_line = axis.plot([], [], label=f"{prefix}-y")[0]
        self.theta_line = axis.plot([], [], label=f"{prefix}-theta")[0]

    def update(self, msg: Odometry):
        self.states.append(msg)
        data = odometries_to_array(self.states)
        self.x_line.set_xdata(data[:, 0])
        self.x_line.set_ydata(data[:, 1])
        self.y_line.set_xdata(data[:, 0])
        self.y_line.set_ydata(data[:, 2])
        self.theta_line.set_xdata(data[:, 0])
        self.theta_line.set_ydata(data[:, 3])


class ScalarLine:
    def __init__(self, axis, label: str) -> None:
        self.times = []
        self.states = []
        self.line = axis.plot([], [], label=label)[0]

    def update(self, timestamp: float, value: float):
        self.times.append(timestamp)
        self.states.append(value)
        self.line.set_xdata(self.times)
        self.line.set_ydata(self.states)


class Plotter:
    def __init__(self) -> None:
        rospy.init_node("plot_generator")

        self.plot_delay = 0.01

        self.init()
        self.forwarded_dist_line = ScalarLine(self.error_over_time_plot, "error")
        self.filter_line = OdometryLine(self.state_over_time_plot, "filter")
        self.odom_line = OdometryLine(self.state_over_time_plot, "odom")

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
            "forwarded_landmark",
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
        distance = Pose2d.from_ros_pose(msg.pose.pose).distance(
            Pose2d.from_ros_pose(self.prev_landmark.pose.pose)
        )
        self.forwarded_dist_line.update(msg.header.stamp.to_sec(), distance)

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
