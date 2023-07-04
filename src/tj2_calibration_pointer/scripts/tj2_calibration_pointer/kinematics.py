import rospy
import numpy as np
import tf.transformations
from urdf_parser_py.urdf import Robot  # type: ignore
from scipy.optimize import minimize  # type: ignore
from typing import Tuple, Optional
from geometry_msgs.msg import Point
from tj2_calibration_pointer.jit_functions import (
    jit_cost_function,
    jit_compute_forward_kinematics,
)


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

        # compile cost function
        jit_cost_function(
            np.array([0.0, 0.0]),
            np.eye(4),
            np.eye(4),
            np.eye(4),
            np.array([0.0, 0.0, 0.0]),
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
        return jit_compute_forward_kinematics(
            pan_angle,
            self.pan_joint,
            tilt_angle,
            self.tilt_joint,
            self.end_effector_joint,
        )

    def cost_function(self, state: np.ndarray, goal_point: np.ndarray) -> float:
        return jit_cost_function(
            state, self.pan_joint, self.tilt_joint, self.end_effector_joint, goal_point
        )

    def compute_inverse_kinematics(
        self, goal: Point, initial_guess: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        goal_point = np.array([goal.x, goal.y, goal.z])
        result = minimize(
            self.cost_function,
            np.array(initial_guess),
            method="SLSQP",
            tol=1e-3,
            args=(goal_point,),
        )
        if not result.success:
            rospy.logwarn(f"Failed to find solution: {result.message}")
            return None
        pan_angle = result.x[0]
        tilt_angle = result.x[1]
        return (pan_angle, tilt_angle)
