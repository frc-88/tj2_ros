import csv
import numpy as np
from scipy.optimize import curve_fit
from matplotlib import pyplot as plt


def fit_curve(x, a, b, c):
    return a * -x * np.log(b * x) + c


with open("limelight_vs_ros.csv") as file:
    reader = csv.reader(file)

    raw_data = []
    for row in reader:
        raw_data.append(list(map(float, row)))
data = np.array(raw_data)
x = data[:, 0]
y = data[:, 1]

popt, pcov = curve_fit(fit_curve, x, y)
y_fit = fit_curve(x, *popt)
print(popt)
plt.figure(1)
plt.plot(x, y_fit, label=f"a={popt[0]:0.4f}, b={popt[1]:0.4f}, c={popt[2]:0.4f}")
plt.plot(x, y)
plt.xlabel("Limelight distance")
plt.ylabel("ROS distance")
plt.legend()

plt.show()
