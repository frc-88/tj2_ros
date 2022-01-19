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


class BouncePredictor:
    def __init__(self):
        self.rho = 0.75
        self.tau = 0.05
        self.g = -9.81
        self.a_friction = -0.1
        self.t_step = 0.001
        self.ground_plane = -0.1

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

        future_state = FilterState(x1, y1, z1, 0.0, vx0, vy0, vz0, 0.0)
        future_state.stamp = state.stamp + t_window
        return future_state
