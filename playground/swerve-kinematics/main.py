import math
import numpy as np
import matplotlib.pyplot as plt
from swerve_kinematics import SwerveKinematics

positions = [
    [1.0, 1.0],
    [-1.0, 1.0],
    [-1.0, -1.0],
    [1.0, -1.0],
]

module_states = np.array([
    [math.radians(40.0), 2.0],
    [math.radians(47.0), 2.1],
    [math.radians(40.0), 2.0],
    [math.radians(45.0), 2.2],
])

swerve = SwerveKinematics(positions)

# print(chassis_speeds)

total_time = 0.0
dt = 0.01
xs = []
ys = []

for time_step in range(100):
    chassis_speeds = swerve.module_to_chassis_speeds(module_states)
    module_states[:, 0] += math.radians(1.0)

    state = swerve.estimate_pose(dt)
    xs.append(state.x)
    ys.append(state.y)
    total_time = time_step * dt

print(swerve.state)

print("Time elapsed: %s" % total_time)

plt.xlim(-5.0, 5.0)
plt.ylim(-5.0, 5.0)
plt.plot(xs, ys, color="blue", marker=".", linestyle="-")
plt.show()
