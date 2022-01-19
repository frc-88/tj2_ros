from matplotlib import pyplot as plt


def get_next_state(t0, x0, v0, a, t_step):
    t1 = t0 + t_step
    dt = t1 - t0

    x1 = x0 + v0 * dt + 0.5 * a * dt * dt
    v1 = v0 + a * dt
    return t1, x1, v1


def one_bounce(t0, x0, v0, a, t_step):
    t_data = []
    x_data = []

    while x0 >= 0.0:
        t0, x0, v0 = get_next_state(t0, x0, v0, a, t_step)
        t_data.append(t0)
        x_data.append(x0)

    return t_data, x_data, v0


def get_bounces(x0, v0, rho, tau, g, t_limit, v_limit, t_step):
    """
    :param x0: initial height
    :param v0: initial velocity
    :param rho: coefficient of restitution (how much velocity is retained after a bounce)
    :param tau: contact time (how long velocity is 0.0 during a bounce)
    :param g: acceleration due to gravity (must be < 0.0)
    :param t_limit: maximum trajectory time window
    :param v_limit: velocity below which trajectory is considered done
    :param t_step: time resolution of simulation
    :return: t data, x data
    """
    assert g < 0.0
    t_data = [0.0]
    x_data = [x0]
    first = True
    while first or (v0 > v_limit or t_data[-1] < t_limit):
        t_data_0, x_data_0, v0 = one_bounce(t_data[-1] + (0.0 if first else tau), x_data[-1], v0, g, t_step)
        t_data.extend(t_data_0)
        x_data.extend(x_data_0)
        x_data[-1] = 0.0
        v0 *= -rho
        if first:
            first = False
    return t_data, x_data


def roll_object(x0, v0, a, t_limit, t_step):
    t_data = []
    x_data = []
    t0 = 0.0

    while t0 < t_limit and v0 > 0.0:
        t0, x0, v0 = get_next_state(t0, x0, v0, a, t_step)
        t_data.append(t0)
        x_data.append(x0)

    while t_data[-1] < t_limit:
        t_data.append(t_data[-1] + t_step)
        x_data.append(x_data[-1])

    return t_data, x_data


def main():
    x0 = 0.0
    z0 = 1.0
    vx0 = 1.0
    vz0 = 0.0
    rho = 0.75
    tau = 0.025
    t_step = 0.0001
    g = -9.81
    a_friction = -2.0
    v_limit = 0.05
    t_limit = 10.0

    tz_data, z_data = get_bounces(z0, vz0, rho, tau, g, t_limit, v_limit, t_step)
    tx_data, x_data = roll_object(x0, vx0, a_friction, t_limit, t_step)

    plt.plot(tz_data, z_data)
    plt.plot(tx_data, x_data)
    plt.show()


if __name__ == '__main__':
    main()
