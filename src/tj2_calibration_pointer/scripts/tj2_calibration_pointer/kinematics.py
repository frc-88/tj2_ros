import rospy
import numpy as np
import tf.transformations
from urdf_parser_py.urdf import Robot, URDF  # type: ignore
from scipy.optimize import minimize  # type: ignore
from typing import Tuple, Optional
from geometry_msgs.msg import Point


class CalibrationPointerKinematics:
    def __init__(
        self,
        robot: Robot,
        pan_link_name: str,
        tilt_link_name: str,
        end_effector_base_link_name: str,
        end_effector_tip_link_name: str,
    ) -> None:
        self.robot = robot
        self.pan_link_name = pan_link_name
        self.tilt_link_name = tilt_link_name
        self.end_effector_base_link_name = end_effector_base_link_name
        self.end_effector_tip_link_name = end_effector_tip_link_name

        self.pan_joint = self.lookup_homogenous_transform(
            self.pan_link_name, self.tilt_link_name
        )
        self.tilt_joint = self.lookup_homogenous_transform(
            self.tilt_link_name, self.end_effector_base_link_name
        )
        self.end_effector_joint = self.lookup_homogenous_transform(
            self.end_effector_base_link_name, self.end_effector_tip_link_name
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

    def compute_forward_kinematics(
        self, pan_angle: float, tilt_angle: float
    ) -> np.ndarray:
        pan_transform = tf.transformations.euler_matrix(0.0, 0.0, pan_angle)
        tilt_transform = tf.transformations.euler_matrix(0.0, 0.0, tilt_angle)
        system_transform = (
            pan_transform
            @ self.pan_joint
            @ tilt_transform
            @ self.tilt_joint
            @ self.end_effector_joint
        )
        return system_transform

    def compute_line(
        self, pan_angle: float, tilt_angle: float, goal_point: np.ndarray
    ) -> np.ndarray:
        system_transform = self.compute_forward_kinematics(pan_angle, tilt_angle)
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

    def compute_inverse_kinematics(
        self, goal: Point, initial_guess: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        goal_point = np.array([goal.x, goal.y, goal.z])
        result = minimize(
            self.cost_function,
            np.array(initial_guess),
            method="SLSQP",
            # tol=1e-6,
            args=(goal_point,),
            # bounds=self.joint_limits,
        )
        if not result.success:
            rospy.logwarn(f"Failed to find solution: {result.message}")
            return None
        pan_angle = result.x[0]
        tilt_angle = result.x[1]
        return (pan_angle, tilt_angle)
