import sys
import numpy as np
from matplotlib import pyplot as plt

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.motion_profile import TrapezoidalProfile, State

controller = TrapezoidalProfile(
    dict(
        kp=1.0,
        ki=0.0,
        kd=0.0,
    ),
    dict(
        max_speed=1.0,
        max_accel=1.0
    )
)

def simulation():
    controller.reset(State(-1.0))
    t = np.linspace(0.0, 5.0, 1000)

    velocities = []
    positions = []
    commands = []

    for time in t:
        state = controller.calculate(time)
        command = controller.calculate_command_velocity(-0.1, time)
        velocities.append(state.velocity)
        positions.append(state.position)
        commands.append(command)

    plt.plot(t, velocities)
    plt.plot(t, positions)
    plt.plot(t, commands)
    plt.show()

def live():
    distances = np.linspace(5.0, 0.0, 5)
    velocities = []
    positions = []
    dt = 0.05
    t = []
    t0 = 0.0
    t1 = 0.0
    for distance in distances:
        controller.reset_position(distance)
        t0 = t1
        for _ in range(10):
            t1 += dt
            state = controller.calculate(t1 - t0)
            velocities.append(state.velocity)
            positions.append(state.position)
            t.append(t1)

    plt.plot(t, velocities)
    plt.plot(t, positions)
    plt.show()

# simulation()
live()