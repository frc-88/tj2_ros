import pandas as pd
import numpy as np
from matplotlib import pyplot as plt


def meters_to_in(meters):
    return meters * 39.37


def in_to_meters(inches):
    return inches * 0.0254


path = "new_vs_old_target_data.csv"
df = pd.read_csv(path)

ax1 = plt.subplot(2, 1, 1)
ax2 = plt.subplot(2, 1, 2)

initial_t = df["timestamp"].loc[0]
timestamps = df["timestamp"] - initial_t

ax1.plot(timestamps, df["old_angle"], color='b', label="old")
ax1.plot(timestamps, df["new_angle"], color='r', label="new")
ax1.plot(timestamps, df["vx"], color='gray', label="vx")
ax1.set_xlabel("timestamp")
ax1.set_ylabel("angle (radians)")
ax1.legend()

ax2.plot(timestamps, df["old_distance"], color='b', label="old")
ax2.plot(timestamps, df["new_distance"], color='r', label="new")
ax2.plot(timestamps, df["vx"], color='gray', label="vx")
ax2.set_xlabel("timestamp")
ax2.set_ylabel("distance (meters)")
ax2.legend()
plt.show()
