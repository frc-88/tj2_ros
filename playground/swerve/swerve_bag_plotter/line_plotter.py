from matplotlib import pyplot as plt
from state_loader import get_states
import numpy as np

# data = get_states("data/diffyjr_2022-06-09-22-42-13_0.json")
# data = get_states("data/diffyjr_2022-06-09-23-07-18_0.json")
data = get_states("data/diffyjr_2022-06-09-23-41-58_0.json")
# data = get_states("data/diffyjr_2022-06-09-23-55-06_0.json")

num_modules = len(data)

fig = plt.figure()
axes = []


for index in range(num_modules * 2):
    ax = fig.add_subplot(4, 2, index + 1)
    ax.set_title("Module %s" % (index // 2))
    ax.set_xlabel("voltage (V)")
    ax.set_ylabel("time (s)")
    axes.append(ax)

for module_index in range(num_modules):
    timestamps = []
    lo_voltages = []
    hi_voltages = []
    lo_voltages_ref = []
    hi_voltages_ref = []
    for index in range(len(data[module_index])):
        row = data[module_index][index]
        hi_voltages.append(row["hi_voltage"])
        lo_voltages.append(row["lo_voltage"])
        hi_voltages_ref.append(row["hi_voltage_ref"])
        lo_voltages_ref.append(row["lo_voltage_ref"])
        timestamps.append(row["time"])

    ax1 = axes[2 * module_index]
    ax2 = axes[2 * module_index + 1]
    ax1.plot(timestamps, hi_voltages, label="hi")
    ax1.plot(timestamps, hi_voltages_ref, label="hi_ref")
    ax2.plot(timestamps, lo_voltages, label="lo")
    ax2.plot(timestamps, lo_voltages_ref, label="lo_ref")

    ax1.legend()
    ax2.legend()
plt.show()
