import os
import csv
import rospkg
import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit
from scipy.interpolate import interp1d
from tj2_tools.robot_state import Pose2d

def read_tof_file(path):
    hood_up_table = []
    hood_down_table = []
    print("Loading tof table from %s" % path)
    with open(path) as file:
        reader = csv.reader(file)
        header = next(reader)
        for row in reader:
            parsed_row = list(map(float, row[1:]))
            if row[0] == "up":
                hood_up_table.append(parsed_row)
            elif row[0] == "down":
                hood_down_table.append(parsed_row)

    hood_up_table.sort(key=lambda x: x[0])
    hood_down_table.sort(key=lambda x: x[0])
    return np.array(hood_up_table), np.array(hood_down_table)

def read_data_file(path):
    hood_up_table = []
    hood_down_table = []
    with open(path) as file:
        reader = csv.reader(file)
        header = next(reader)
        for row in reader:
            if row[0].strip().startswith("#"):
                continue
            if row[0] != "tof":
                continue
            hood = row[header.index("hood")]
            row = [
                float(row[header.index("distance")]),
                float(row[header.index("value")])
            ]
            if hood == "up":
                hood_up_table.append(row)
            elif hood =="down":
                hood_down_table.append(row)
    return np.array(hood_up_table), np.array(hood_down_table)


def fit_curve_fn(x, a, b):
    return a * x + b

def fit_tof_data(x_fit, x, y):
    popt, pcov = curve_fit(fit_curve_fn, x, y)
    y_fit = fit_curve_fn(x_fit, *popt)
    print(popt)
    return y_fit

def create_interp(table):
    if len(table) == 0:
        return None
    x_samples = table[:, 0]
    y_samples = table[:, 1]
    return interp1d(x_samples, y_samples, kind="linear", bounds_error=False, fill_value=0.0)


def main():
    rospack = rospkg.RosPack()
    tof_path = os.path.join(rospack.get_path("tj2_target"), "config", "time_of_flight.csv")
    recorded_data_path = os.path.join(rospack.get_path("tj2_target"), "config", "recorded_data.csv")
    hood_up_table, hood_down_table = read_tof_file(tof_path)
    # hood_up_table, hood_down_table = read_data_file(recorded_data_path)
    traj_interp_up = create_interp(hood_up_table)
    traj_interp_down = create_interp(hood_down_table)
    x_dist = np.linspace(0.0, 10.0, 100)
    up_tof = traj_interp_up(x_dist)
    down_tof = traj_interp_down(x_dist)

    hood_up_fit = fit_tof_data(x_dist, hood_up_table[:, 0], hood_up_table[:, 1])
    hood_down_fit = fit_tof_data(x_dist, hood_down_table[:, 0], hood_down_table[:, 1])

    line = plt.plot(hood_up_table[:, 0], hood_up_table[:, 1], '.')[0]
    # plt.plot(x_dist, up_tof, color=line.get_color())
    plt.plot(x_dist, hood_up_fit, color=line.get_color())
    line = plt.plot(hood_down_table[:, 0], hood_down_table[:, 1], '.')[0]
    # plt.plot(x_dist, down_tof, color=line.get_color())
    plt.plot(x_dist, hood_down_fit, color=line.get_color())
    plt.show()

main()
