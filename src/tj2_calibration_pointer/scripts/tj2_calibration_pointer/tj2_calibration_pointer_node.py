#!/usr/bin/env python3
from typing import Tuple
import numpy as np
import tf.transformations
import rospy
import tf2_ros
from scipy.optimize import minimize, LinearConstraint  # type: ignore
from std_msgs.msg import Bool, Float64
from urdf_parser_py.urdf import URDF  # type: ignore
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from tj2_tools.transforms import transform_pose


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

        self.is_active = True
        self.goal_point = Point(x=0.0, y=0.0, z=0.0)
        self.pan_joint_command = 0.0
        self.tilt_joint_command = 0.0

        self.joint_limits = LinearConstraint(
            np.eye(2),
            np.array([-np.pi / 2.0, -np.pi / 2.0]),
            np.array([np.pi / 2.0, np.pi / 2.0]),
        )

        rospy.loginfo(f"Loading URDF from {self.urdf_key}")
        self.robot = URDF.from_parameter_server(self.urdf_key)

        self.pan_joint = self.lookup_homogenous_transform(
            self.pan_link_name, self.tilt_link_name
        )
        self.tilt_joint = self.lookup_homogenous_transform(
            self.tilt_link_name, self.end_effector_base_link_name
        )
        self.end_effector_joint = self.lookup_homogenous_transform(
            self.end_effector_base_link_name, self.end_effector_tip_link_name
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

    def lookup_homogenous_transform(
        self, parent_link: str, child_link: str
    ) -> np.ndarray:
        chain = self.robot.get_chain(
            parent_link, child_link, joints=True, links=False, fixed=True
        )
        homogenous_transform = np.identity(4)
        for joint_name in chain:
            joint = self.robot.joint_map[joint_name]
            homogenous_transform = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(joint.origin.xyz),
                tf.transformations.euler_matrix(*joint.origin.rpy),
                homogenous_transform,
            )
        return np.array(homogenous_transform)

    def compute_system_transform(
        self, pan_angle: float, tilt_angle: float
    ) -> np.ndarray:
        pan_transform = tf.transformations.euler_matrix(0.0, 0.0, pan_angle)
        tilt_transform = tf.transformations.euler_matrix(0.0, 0.0, tilt_angle)
        system_transform = (
            self.end_effector_joint
            @ self.tilt_joint
            @ tilt_transform
            @ self.pan_joint
            @ pan_transform
        )
        return system_transform

    def compute_line(
        self, pan_angle: float, tilt_angle: float, goal_point: np.ndarray
    ) -> np.ndarray:
        system_transform = self.compute_system_transform(pan_angle, tilt_angle)
        line_vector = np.array([np.linalg.norm(goal_point), 0.0, 0.0, 1.0])
        line_transform = system_transform @ line_vector

        line_pt1 = tf.transformations.translation_from_matrix(system_transform)
        line_pt2 = line_transform[0:3]
        return np.array([line_pt1, line_pt2])

    def distance_to_point(self, point1: np.ndarray, point2: np.ndarray) -> float:
        return float(np.linalg.norm(point2 - point1, axis=0))

    def cost_function(self, state: np.ndarray, goal_point: np.ndarray) -> float:
        pan_angle = state[0]
        tilt_angle = state[1]
        line = self.compute_line(pan_angle, tilt_angle, goal_point)
        return self.distance_to_point(line[1], goal_point)

    def compute_joint_angles(
        self, goal: Point, initial_guess: Tuple[float, float]
    ) -> Tuple[float, float]:
        rospy.loginfo(f"Goal: {goal.x}, {goal.y}, {goal.z}. guess: {initial_guess}")
        goal_point = np.array([goal.x, goal.y, goal.z])
        result = minimize(
            self.cost_function,
            np.array(initial_guess),
            method="Nelder-Mead",
            # tol=1e-6,
            args=(goal_point,),
            # constraints=self.joint_limits,
        )
        pan_angle = result.x[0]
        tilt_angle = result.x[1]
        return (pan_angle, tilt_angle)

    def goal_callback(self, goal_point: PointStamped) -> None:
        goal_pose = PoseStamped()
        goal_pose.header = goal_point.header
        goal_pose.pose.position = goal_point.point
        goal_pose.pose.orientation.w = 1.0
        transform = transform_pose(self.tf_buffer, goal_pose, self.pan_link_name)
        if transform is not None:
            self.goal_point = transform.pose.position
        else:
            rospy.logwarn(
                "Could not transform goal point to device root frame. Ignoring goal."
            )

    def set_active_callback(self, is_active: Bool) -> None:
        self.is_active = is_active.data

    def publish_goal_from_joints(self, pan_angle: float, tilt_angle: float) -> None:
        system_transform = self.compute_system_transform(pan_angle, tilt_angle)
        end_effector_point = system_transform @ np.array([0.0, 0.0, 0.0, 1.0])
        goal_point = PointStamped()
        goal_point.header.frame_id = self.pan_link_name
        goal_point.point = Point(
            x=end_effector_point[0], y=end_effector_point[1], z=end_effector_point[2]
        )
        self.goal_publisher.publish(goal_point)

    def run(self) -> None:
        rate = rospy.Rate(self.update_rate)
        prev_goal = self.goal_point

        while not rospy.is_shutdown():
            rate.sleep()
            # if self.is_active and prev_goal != self.goal_point:
            # prev_goal = self.goal_point
            # (
            #     self.pan_joint_command,
            #     self.tilt_joint_command,
            # ) = self.compute_joint_angles(
            #     self.goal_point,
            #     (self.pan_joint_command, self.tilt_joint_command),
            # )
            # rospy.loginfo(
            #     f"pan={self.pan_joint_command}, tilt={self.tilt_joint_command}"
            # )
            self.publish_goal_from_joints(
                self.pan_joint_command, self.tilt_joint_command
            )
            self.pan_joint_pub.publish(self.pan_joint_command)
            self.tilt_joint_pub.publish(self.tilt_joint_command)


if __name__ == "__main__":
    node = TJ2CalibrationPointer()
    node.run()
