import pandas as pd
import numpy as np
from matplotlib import pyplot as plt


def meters_to_in(meters):
    return meters * 39.37


def in_to_meters(inches):
    return inches * 0.0254


a = 2.69840693e-01
b = 1.25456722e-04
c = -1.63681970e+01

def limelight_correction(x):
    return a * -x * np.log(b * x) + c


# path = "target_data.csv"
path = "target_data_2_ball.csv"
df = pd.read_csv(path)

ax1 = plt.subplot(2, 1, 1)
ax2 = plt.subplot(2, 1, 2)

initial_t = df["timestamp"].loc[0]
timestamps = df["timestamp"] - initial_t

ax1.plot(timestamps, np.degrees(df["target_angle"]), color='b', label="ROS")
ax1.plot(timestamps, np.degrees(df["limelight_angle"]), color='g', label="limelight")
ax1.set_xlabel("timestamp")
ax1.set_ylabel("angle (degrees)")
ax1.legend()

ax2.plot(timestamps, meters_to_in(df["target_distance"]), color='b', label="ROS")
ax2.plot(timestamps, meters_to_in(df["limelight_distance"]), color='g', label="limelight")
ax2.plot(timestamps, limelight_correction(meters_to_in(df["limelight_distance"])), color='limegreen', label="corrected")
ax2.set_xlabel("timestamp")
ax2.set_ylabel("distance (inches)")
ax2.legend()
plt.show()
