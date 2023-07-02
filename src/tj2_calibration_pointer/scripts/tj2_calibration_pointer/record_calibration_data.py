#!/usr/bin/env python3
import itertools
import numpy as np
from networktables import NetworkTables
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Time, Float64
from geometry_msgs.msg import PointStamped, Point
from typing import List
import rospy


class CalibrationRecorder:
    def __init__(self) -> None:
        self.node_name = "record_calibration_data"
        rospy.init_node(self.node_name)

        self.nt_host = rospy.get_param("~nt_host", "10.0.88.2")
        NetworkTables.initialize(server=self.nt_host)
        self.root_key = "/"
        self.nt = NetworkTables.getTable(self.root_key)

        self.pan_state_key = "SmartDashboard/CalibrationPointer/state/pan"
        self.tilt_state_key = "SmartDashboard/CalibrationPointer/state/tilt"
        self.preference_key_format = (
            "Preferences/CalibrationPointer/{device}_servo_{type}_{index}"
        )

        self.index_cycler = itertools.cycle([1, 2])

        self.pan_angle = 0.0
        self.tilt_angle = 0.0

        self.start_time = rospy.Time.now()

        self.tag_subscriber = rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=1
        )
        self.pan_joint_sub = rospy.Subscriber(
            "pan_joint", Float64, self.pan_joint_callback, queue_size=1
        )
        self.tilt_joint_sub = rospy.Subscriber(
            "tilt_joint", Float64, self.tilt_joint_callback, queue_size=1
        )
        self.goal_publisher = rospy.Publisher("goal", PointStamped, queue_size=1)
        self.record_subscriber = rospy.Subscriber(
            "record", Time, self.record_callback, queue_size=1
        )

    def get_pan_value(self) -> float:
        return self.nt.getNumber(self.pan_state_key, 0.0)

    def get_tilt_value(self) -> float:
        return self.nt.getNumber(self.tilt_state_key, 0.0)

    def save_preference(
        self, device: str, value_type: str, index: int, value: float
    ) -> None:
        key = self.preference_key_format.format(
            device=device, type=value_type, index=index
        )
        rospy.loginfo(f"Saving {key} = {value}")
        self.nt.putNumber(key, value)

    def tag_callback(self, msg: AprilTagDetectionArray) -> None:
        points: List[Point] = []
        for detection in msg.detections:
            points.append(detection.pose.pose.pose.position)
        closest_point = min(points, key=lambda p: np.linalg.norm([p.x, p.y, p.z]))

        goal = PointStamped()
        goal.point = closest_point
        goal.header = msg.header

        self.goal_publisher.publish(goal)

    def record_callback(self, msg: Time) -> None:
        if msg.data < self.start_time:
            return
        index = next(self.index_cycler)
        self.save_preference("pan", "angle", index, self.pan_angle)
        self.save_preference("tilt", "angle", index, self.tilt_angle)
        self.save_preference("pan", "value", index, self.get_pan_value())
        self.save_preference("tilt", "value", index, self.get_tilt_value())
        rospy.loginfo(f"Recorded data for index {index}")

    def pan_joint_callback(self, msg: Float64) -> None:
        self.pan_angle = msg.data

    def tilt_joint_callback(self, msg: Float64) -> None:
        self.tilt_angle = msg.data

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    node = CalibrationRecorder()
    node.run()
