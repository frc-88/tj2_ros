import time
from matplotlib import pyplot as plt
from numba import njit
import numpy as np
import tf.transformations
from scipy.optimize import minimize  # type: ignore


@njit
def distance_to_line(p: np.ndarray, q: np.ndarray, r: np.ndarray):
    x = p - q
    return np.linalg.norm(np.dot(r - q, x) / np.dot(x, x) * (p - q) + q - r)


PAN_JOINT = tf.transformations.concatenate_matrices(
    tf.transformations.translation_matrix([0.01225, 0.02650, 0.0]),
    tf.transformations.quaternion_matrix([0.0, 0.0, 0.0, 1.0]),
)
TILT_JOINT = tf.transformations.concatenate_matrices(
    tf.transformations.translation_matrix([0.015, -0.01, 0.0]),
    tf.transformations.quaternion_matrix([0.7071, 0.0, 0.0, 0.7071]),
)

END_EFFECTOR_JOINT = tf.transformations.concatenate_matrices(
    tf.transformations.translation_matrix([0.0, 0.0, 0.0]),
    tf.transformations.quaternion_matrix([0.0, 0.7071, 0.0, 0.7071]),
)


def compute_system_transform(pan_angle: float, tilt_angle: float):
    pan_transform = tf.transformations.euler_matrix(0.0, 0.0, pan_angle)
    tilt_transform = tf.transformations.euler_matrix(0.0, 0.0, tilt_angle)
    system_transform = (
        END_EFFECTOR_JOINT @ tilt_transform @ TILT_JOINT @ pan_transform @ PAN_JOINT
    )
    return system_transform


def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def plot_angle(pan_angle: float, tilt_angle: float, goal_point: np.ndarray) -> None:
    system_transform = compute_system_transform(pan_angle, tilt_angle)
    line_vector = np.array([1.0, 0.0, 0.0, 1.0])
    line_transform = system_transform @ line_vector

    line_pt1 = tf.transformations.translation_from_matrix(system_transform)
    line_pt2 = line_transform[0:3]
    line = np.array([line_pt1, line_pt2]).T

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(*line)
    ax.scatter(*goal_point)
    set_axes_equal(ax)
    plt.show()


def main():
    goal_point = np.array([1.0, 1.0, 1.0])

    def cost_function(state: np.ndarray) -> float:
        nonlocal goal_point
        pan_angle = state[0]
        tilt_angle = state[1]
        system_transform = compute_system_transform(pan_angle, tilt_angle)
        line_vector = np.array([1.0, 0.0, 0.0, 1.0])
        line_transform = system_transform @ line_vector

        line_pt1 = tf.transformations.translation_from_matrix(system_transform)
        line_pt2 = line_transform[0:3]
        return distance_to_line(line_pt1, line_pt2, np.array([goal_point]))

    cost_function(np.array([0.5, 0.5]))

    x0 = np.array([0.5, 0.5])

    t0 = time.monotonic()
    result = minimize(cost_function, x0, method="Nelder-Mead", tol=1e-6)
    t1 = time.monotonic()
    print(f"Solve took {t1 - t0}")
    pan_angle = result.x[0]
    tilt_angle = result.x[1]
    print(f"{pan_angle=}, {tilt_angle=}")
    plot_angle(pan_angle, tilt_angle, goal_point)


if __name__ == "__main__":
    main()
