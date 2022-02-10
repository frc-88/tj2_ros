import math
from tj2_tools.particle_filter.state import FilterState


def get_next_state(t0, x0, v0, a, t_step):
    t1 = t0 + t_step
    dt = t1 - t0

    x1 = x0 + v0 * dt + 0.5 * a * dt * dt
    v1 = v0 + a * dt
    return t1, x1, v1


def one_bounce(t0, x0, v0, a, t_limit, t_step):
    while x0 >= 0.0 and t0 < t_limit:
        t0, x0, v0 = get_next_state(t0, x0, v0, a, t_step)

    if x0 < 0.0:
        x0 = 0.0

    return t0, x0, v0


def get_bounces(x0, v0, rho, tau, g, t_limit, t_step):
    """
    :param x0: initial height
    :param v0: initial velocity
    :param rho: coefficient of restitution (how much velocity is retained after a bounce)
    :param tau: contact time (how long velocity is 0.0 during a bounce)
    :param g: acceleration due to gravity (must be < 0.0)
    :param t_limit: time above which the simulation is considered done
    :param t_step: time resolution of simulation
    :return: t data, x data
    """
    assert g < 0.0
    t1 = 0.0
    x1 = x0
    first = True
    while first or t1 < t_limit:
        t1, x1, v0 = one_bounce(t1 + (0.0 if first else tau), x1, v0, g, t_limit, t_step)
        v0 *= -rho
        if first:
            first = False
    return x1, v0


def roll_object(x0, v0, a, t_limit, t_step):
    t0 = 0.0

    while t0 < t_limit and v0 > 0.0:
        t0, x0, v0 = get_next_state(t0, x0, v0, a, t_step)

    return x0, v0


def get_time_to_distance(x1, v0, a, vmax, t_limit, t_step, data=None):
    """
    :param x1: distance to goal (assume starting from 0.0)
    :param v0: initial velocity
    :param a: acceleration of system
    :param vmax: maximum velocity of the system
    :param t_limit: time above which the simulation is considered done
    :param t_step: time resolution of simulation
    :return: t arrival
    """
    t0 = 0.0
    x0 = 0.0
    a = math.copysign(a, x1)

    while t0 < t_limit:
        t0, x0, v0 = get_next_state(t0, x0, v0, a, t_step)
        v0 = min(vmax, max(-vmax, v0))  # clip velocity to -vmax...+vmax
        if data is not None:
            data.append([t0, x0, v0])
        if x1 < 0.0:
            if x0 < x1:
                return t0
        else:
            if x0 > x1:
                return t0

    return t0


class BouncePredictor:
    def __init__(self, rho, tau, g, a_friction, t_step, ground_plane, a_robot, v_max_robot, t_limit):
        self.rho = rho
        self.tau = tau
        self.g = g
        self.a_friction = a_friction
        self.t_step = t_step
        self.ground_plane = ground_plane
        self.a_robot = a_robot
        self.v_max_robot = v_max_robot
        self.t_limit = t_limit

    def get_prediction(self, state: FilterState, t_window):
        x0 = state.x
        vx0 = state.vx
        x1, vx1 = roll_object(x0, vx0, self.a_friction, t_window, self.t_step)

        y0 = state.y
        vy0 = state.vy
        y1, vy1 = roll_object(y0, vy0, self.a_friction, t_window, self.t_step)

        z0 = state.z - self.ground_plane
        vz0 = state.vz
        z1, vz1 = get_bounces(z0, vz0, self.rho, self.tau, self.g, t_window, self.t_step)
        z1 += self.ground_plane

        future_state = FilterState(x1, y1, z1, state.theta, vx1, vy1, vz1, state.vt)
        future_state.stamp = state.stamp + t_window
        return future_state

    def get_robot_intersection(self, robot_state: FilterState, obj_state: FilterState):
        # plot a course to where the object will head
        # given robot parameters, find where the robot and object intersect
        robot_state_2d = FilterState.from_state(robot_state)
        obj_state_2d = FilterState.from_state(obj_state)
        robot_state_2d.z = 0.0
        robot_state_2d.vz = 0.0
        obj_state_2d.z = 0.0
        obj_state_2d.vz = 0.0

        obj_dist = obj_state_2d.distance(robot_state_2d)

        t_window = get_time_to_distance(obj_dist, robot_state_2d.velocity_magnitude(), self.a_robot, self.v_max_robot,
                                        self.t_limit, self.t_step)
        return self.get_prediction(obj_state, t_window)
