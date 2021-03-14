import math
import numpy as np
import matplotlib.pyplot as plt
from swerve_kinematics import SwerveKinematics
from swerve_log_parser import SwerveLogParser

positions = [
    [-0.9159166667, 0.5204583333],
    [-0.9159166667, -0.5204583333],
    [0.9159166667, -0.5204583333],
    [0.9159166667, 0.5204583333],
]

# path = "/home/woz4tetra/Desktop/swerve-1.txt"
path = "/home/woz4tetra/Desktop/swerve-figure-8.txt"

parser = SwerveLogParser(4, path)

# module_states = np.array([
#     [math.radians(147.48039562500003), 2.654882421144346],
#     [math.radians(-147.30468075), 2.4725700248691274],
#     [math.radians(-26.27931375000003), 2.4520560757574454],
#     [math.radians(31.552610625), 2.6558563771378134],
# ])

swerve = SwerveKinematics(positions)

calculated_pose = [[], []]
calculated_speeds = [[], []]
calculated_pose_times = []
recorded_pose = [[], []]

prev_time = parser.states[0][0]
for timestamp, module_states in parser.states:
    chassis_speeds = swerve.module_to_chassis_speeds(module_states.state)
    dt = timestamp - prev_time
    prev_time = timestamp

    state = swerve.estimate_pose(dt)
    calculated_pose[0].append(state.x)
    calculated_pose[1].append(state.y)
    calculated_pose_times.append(timestamp)
    calculated_speeds[0].append(state.vx)
    calculated_speeds[1].append(state.vy)

recorded_x = 0.0
recorded_y = 0.0
for row in parser.logged_poses:
    dx = row[0]
    dy = row[1]
    recorded_x += dx
    recorded_y += dy

    recorded_pose[0].append(recorded_x)
    recorded_pose[1].append(recorded_y)

print(swerve.state)

plt.figure(1)
plt.xlim(-5.0, 5.0)
plt.ylim(-5.0, 5.0)
plt.plot(calculated_pose[0], calculated_pose[1], color="blue", marker="x", linestyle="-", label="calculated")
plt.plot(recorded_pose[0], recorded_pose[1], color="red", marker="x", linestyle="-", label="recorded")
plt.xlabel("X m/s")
plt.ylabel("Y m/s")
plt.title("Calculated vs. logged position")
plt.legend()

plt.figure(2)
plt.plot(calculated_pose_times, calculated_speeds[0], color="blue", marker="x", linestyle="-", label="vx")
plt.plot(calculated_pose_times, calculated_speeds[1], color="red", marker="x", linestyle="-", label="vy")
plt.xlabel("timestamp (sec)")
plt.ylabel("velocity (m/s)")
plt.title("Chassis velocities over time")
plt.legend()

plt.show()
