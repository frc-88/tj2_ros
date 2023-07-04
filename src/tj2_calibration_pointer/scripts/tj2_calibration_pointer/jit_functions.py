import math
from numba import njit
import numpy as np


@njit
def jit_euler_matrix(ai, aj, ak):
    i = 0
    j = 1
    k = 2

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk

    M = np.identity(4)
    M[i, i] = cj * ck
    M[i, j] = sj * sc - cs
    M[i, k] = sj * cc + ss
    M[j, i] = cj * sk
    M[j, j] = sj * ss + cc
    M[j, k] = sj * cs - sc
    M[k, i] = -sj
    M[k, j] = cj * si
    M[k, k] = cj * ci
    return M


@njit
def jit_compute_forward_kinematics(
    pan_angle: float,
    pan_joint: np.ndarray,
    tilt_angle: float,
    tilt_joint: np.ndarray,
    end_effector_joint: np.ndarray,
) -> np.ndarray:
    pan_transform = jit_euler_matrix(0.0, 0.0, pan_angle)
    tilt_transform = jit_euler_matrix(0.0, 0.0, tilt_angle)
    system_transform = (
        pan_transform @ pan_joint @ tilt_transform @ tilt_joint @ end_effector_joint
    )
    return system_transform


@njit
def jit_compute_line(
    pan_angle: float,
    pan_joint: np.ndarray,
    tilt_angle: float,
    tilt_joint: np.ndarray,
    end_effector_joint: np.ndarray,
    goal_point: np.ndarray,
) -> np.ndarray:
    system_transform = jit_compute_forward_kinematics(
        pan_angle, pan_joint, tilt_angle, tilt_joint, end_effector_joint
    )
    line_vector = np.array([np.linalg.norm(goal_point), 0.0, 0.0, 1.0])
    line_transform = system_transform @ line_vector

    line = np.zeros((2, 3))
    line[0, :] = system_transform[:3, 3]
    line[1, :] = line_transform[0:3]
    return line


@njit
def jit_point_to_distance(point1: np.ndarray, point2: np.ndarray) -> float:
    return float(np.linalg.norm(point2 - point1))


@njit
def jit_cost_function(
    state: np.ndarray,
    pan_joint: np.ndarray,
    tilt_joint: np.ndarray,
    end_effector_joint: np.ndarray,
    goal_point: np.ndarray,
) -> float:
    pan_angle = state[0]
    tilt_angle = state[1]
    line = jit_compute_line(
        pan_angle,
        pan_joint,
        tilt_angle,
        tilt_joint,
        end_effector_joint,
        goal_point,
    )
    return jit_point_to_distance(line[1], goal_point)
