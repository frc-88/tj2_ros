import os
import sys
import csv
import rospkg
import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import curve_fit
from scipy.interpolate import interp1d

def read_tof_file(path):
    table = []
    print("Loading tof table from %s" % path)
    with open(path) as file:
        reader = csv.reader(file)
        header = next(reader)
        for row in reader:
            parsed_row = list(map(float, row[1:]))
            table.append(parsed_row)

    table.sort(key=lambda x: x[0])
    return np.array(table)

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
    print("TOF fit constants. Paste the following into your target config")
    print("---------")
    print(f"tof_a_const: {popt[0]:0.8f}  # slope of hood up distance to TOF function")
    print(f"tof_b_const: {popt[1]:0.8f}  # y intercept of hood up distance to TOF function")
    print("---------")
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
    # recorded_data_path = os.path.join(rospack.get_path("tj2_target"), "config", "recorded_data.csv")
    table = read_tof_file(tof_path)
    # hood_up_table, hood_down_table = read_data_file(recorded_data_path)
    traj_interp = create_interp(table)
    x_dist = np.linspace(0.0, 10.0, 100)
    tof = traj_interp(x_dist)

    fit = fit_tof_data(x_dist, table[:, 0], table[:, 1])

    if "show" in sys.argv:
        line = plt.plot(table[:, 0], table[:, 1], '.')[0]
        plt.plot(x_dist, tof, label="TOF from distance")
        plt.plot(x_dist, fit, label="Fitted TOF")
        plt.show()

main()
