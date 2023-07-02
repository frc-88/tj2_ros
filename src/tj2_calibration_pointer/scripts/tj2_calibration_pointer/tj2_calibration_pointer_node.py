#!/usr/bin/env python3
from typing import Optional
import numpy as np
import rospy
import tf2_ros
from std_msgs.msg import Bool, Float64
from urdf_parser_py.urdf import URDF  # type: ignore
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from tj2_tools.transforms import transform_pose
from tj2_calibration_pointer.kinematics import CalibrationPointerKinematics


class TJ2CalibrationPointer:
    def __init__(self) -> None:
        self.node_name = "tj2_calibration_pointer"
        rospy.init_node(self.node_name)
        self.urdf_key = rospy.get_param("~robot_description_param", "robot_description")
        self.pan_link_name = rospy.get_param("~pan_link", "pointer_pan_base")
        self.tilt_link_name = rospy.get_param("~tilt_link", "pointer_tilt_base")
        self.end_effector_base_link_name = rospy.get_param(
            "~end_effector_base_link", "pointer_end_effector_base"
        )
        self.end_effector_tip_link_name = rospy.get_param(
            "~end_effector_tip_link", "pointer_end_effector_tip"
        )
        self.update_rate = rospy.get_param("~update_rate", 30.0)
        self.world_frame = rospy.get_param("~world_frame", "odom")

        self.is_active = True
        self.goal_point = None
        self.pan_joint_command = 0.0
        self.tilt_joint_command = 0.0

        self.initial_guess = (0.5, 0.5)

        rospy.loginfo(f"Loading URDF from {self.urdf_key}")
        self.robot = URDF.from_parameter_server(self.urdf_key)

        self.kinematics = CalibrationPointerKinematics(
            self.robot,
            self.pan_link_name,
            self.tilt_link_name,
            self.end_effector_base_link_name,
            self.end_effector_tip_link_name,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pan_joint_pub = rospy.Publisher("pan_joint", Float64, queue_size=1)
        self.tilt_joint_pub = rospy.Publisher("tilt_joint", Float64, queue_size=1)

        self.goal_subscriber = rospy.Subscriber(
            "goal", PointStamped, self.goal_callback, queue_size=1
        )
        self.goal_publisher = rospy.Publisher(
            "transformed_goal", PointStamped, queue_size=1
        )
        self.set_active_subscriber = rospy.Subscriber(
            "set_active", Bool, self.set_active_callback, queue_size=1
        )

    def goal_callback(self, goal_point: PointStamped) -> None:
        self.goal_point = self.get_transformed_goal_point(goal_point, self.world_frame)

    def get_transformed_goal_point(
        self, point: PointStamped, destination_frame: str
    ) -> Optional[PointStamped]:
        pose = PoseStamped()
        pose.header = point.header
        pose.pose.position = point.point
        pose.pose.orientation.w = 1.0
        transformed_pose = transform_pose(self.tf_buffer, pose, destination_frame)
        if transformed_pose is not None:
            transformed_point = PointStamped()
            transformed_point.header.frame_id = destination_frame
            transformed_point.point = transformed_pose.pose.position
            return transformed_point
        else:
            rospy.logwarn(
                "Could not transform goal point to device root frame. Ignoring goal."
            )

    def set_active_callback(self, is_active: Bool) -> None:
        self.is_active = is_active.data

    def publish_goal_from_joints(self, pan_angle: float, tilt_angle: float) -> None:
        system_transform = self.kinematics.compute_forward_kinematics(
            pan_angle, tilt_angle
        )
        end_effector_point = system_transform @ np.array([0.0, 0.0, 0.0, 1.0])
        goal_point = PointStamped()
        goal_point.header.frame_id = self.pan_link_name
        goal_point.point = Point(
            x=end_effector_point[0], y=end_effector_point[1], z=end_effector_point[2]
        )
        self.goal_publisher.publish(goal_point)

    def run(self) -> None:
        rate = rospy.Rate(self.update_rate)
        prev_goal = None

        while not rospy.is_shutdown():
            rate.sleep()
            if self.goal_point is not None:
                goal_point = self.get_transformed_goal_point(
                    self.goal_point, self.pan_link_name
                )
            else:
                goal_point = None
            if self.is_active and prev_goal != goal_point and goal_point is not None:
                # print(
                #     f"x={goal_point.point.x:0.4f}, y={goal_point.point.y:0.4f}, z={goal_point.point.z:0.4f}"
                # )
                prev_goal = goal_point
                if self.initial_guess is None:
                    initial_guess = (self.pan_joint_command, self.tilt_joint_command)
                else:
                    initial_guess = self.initial_guess
                result = self.kinematics.compute_inverse_kinematics(
                    goal_point.point, initial_guess
                )
                if result is None:
                    continue
                self.pan_joint_command, self.tilt_joint_command = result
                self.publish_goal_from_joints(
                    self.pan_joint_command, self.tilt_joint_command
                )
                self.pan_joint_pub.publish(self.pan_joint_command)
                self.tilt_joint_pub.publish(self.tilt_joint_command)


if __name__ == "__main__":
    node = TJ2CalibrationPointer()
    node.run()
