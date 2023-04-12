import cv2
from matplotlib import pyplot as plt
from matplotlib.pyplot import Axes
import numpy as np
from tj2_tools.occupancy_grid import OccupancyGridManager

blue_grid_zone_string = """
CONE,HIGH,633.680000,20.000000,46.000000
CUBE,HIGH,633.680000,42.000000,35.500000
CONE,HIGH,633.680000,64.000000,46.000000
CONE,HIGH,633.680000,86.000000,46.000000
CUBE,HIGH,633.680000,108.000000,35.500000
CONE,HIGH,633.680000,130.000000,46.000000
CONE,HIGH,633.680000,152.000000,46.000000
CUBE,HIGH,633.680000,174.000000,35.500000
CONE,HIGH,633.680000,196.000000,46.000000
CONE,MIDDLE,616.650000,20.000000,34.000000
CUBE,MIDDLE,616.650000,42.000000,23.530000
CONE,MIDDLE,616.650000,64.000000,34.000000
CONE,MIDDLE,616.650000,86.000000,34.000000
CUBE,MIDDLE,616.650000,108.000000,23.530000
CONE,MIDDLE,616.650000,130.000000,34.000000
CONE,MIDDLE,616.650000,152.000000,34.000000
CUBE,MIDDLE,616.650000,174.000000,23.530000
CONE,MIDDLE,616.650000,196.000000,34.000000
EITHER,LOW,601.090000,20.000000,0.000000
EITHER,LOW,601.090000,42.000000,0.000000
EITHER,LOW,601.090000,64.000000,0.000000
EITHER,LOW,601.090000,86.000000,0.000000
EITHER,LOW,601.090000,108.000000,0.000000
EITHER,LOW,601.090000,130.000000,0.000000
EITHER,LOW,601.090000,152.000000,0.000000
EITHER,LOW,601.090000,174.000000,0.000000
EITHER,LOW,601.090000,196.000000,0.000000
"""

red_grid_zone_string = """
CONE,HIGH,14.320000,20.000000,46.000000
CUBE,HIGH,14.320000,42.000000,35.500000
CONE,HIGH,14.320000,64.000000,46.000000
CONE,HIGH,14.320000,86.000000,46.000000
CUBE,HIGH,14.320000,108.000000,35.500000
CONE,HIGH,14.320000,130.000000,46.000000
CONE,HIGH,14.320000,152.000000,46.000000
CUBE,HIGH,14.320000,174.000000,35.500000
CONE,HIGH,14.320000,196.000000,46.000000
CONE,MIDDLE,31.350000,20.000000,34.000000
CUBE,MIDDLE,31.350000,42.000000,23.530000
CONE,MIDDLE,31.350000,64.000000,34.000000
CONE,MIDDLE,31.350000,86.000000,34.000000
CUBE,MIDDLE,31.350000,108.000000,23.530000
CONE,MIDDLE,31.350000,130.000000,34.000000
CONE,MIDDLE,31.350000,152.000000,34.000000
CUBE,MIDDLE,31.350000,174.000000,23.530000
CONE,MIDDLE,31.350000,196.000000,34.000000
EITHER,LOW,46.910000,20.000000,0.000000
EITHER,LOW,46.910000,42.000000,0.000000
EITHER,LOW,46.910000,64.000000,0.000000
EITHER,LOW,46.910000,86.000000,0.000000
EITHER,LOW,46.910000,108.000000,0.000000
EITHER,LOW,46.910000,130.000000,0.000000
EITHER,LOW,46.910000,152.000000,0.000000
EITHER,LOW,46.910000,174.000000,0.000000
EITHER,LOW,46.910000,196.000000,0.000000
"""

transform_string = """
input: 1.000000,0.000000,0.000000
corner: 9.270000,4.010000
game object: 364.960630,157.874016,0.000000
"""


def plot_grid_zone(ax: Axes, string, color):
    points = []
    for line in string.splitlines():
        if len(line.strip()) == 0:
            continue
        keys = line.split(",")
        object_type = keys[0]
        name = keys[1]
        x = float(keys[2])
        y = float(keys[3])
        z = float(keys[4])
        points.append([x, y, z])
        # ax.annotate(f"{object_type}-{name}", (x, y))
    points = np.array(points)
    ax.scatter(points[:, 0], points[:, 1], color=color)

def plot_object(ax: Axes, string):
    data = {}
    for line in string.splitlines():
        if len(line.strip()) == 0:
            continue
        key, values = line.split(":")
        values = list(map(float, values.split(",")))
        data[key] = values
    x = data["game object"][0]
    y = data["game object"][1]
    ax.plot([x], [y], marker='x', color='g', markersize=10)


def meters_to_inches(meter):
    return meter * 39.3701

def to_corner_coordinates(x, y):
    return x + 8.27, y + 4.01

fig = plt.figure(1)
ax = fig.add_subplot()

ogm = OccupancyGridManager.from_map_file("/root/tj2_ros/src/tj2_laser_slam/maps/charged-up-2023.yaml")
image = ogm.to_debug_image()
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
x0, y0 = to_corner_coordinates(*ogm.get_world_x_y(0, 0))
x1, y1 = to_corner_coordinates(*ogm.get_world_x_y(ogm.width, ogm.height))

x0 = meters_to_inches(x0)
x1 = meters_to_inches(x1)
y0 = meters_to_inches(y0)
y1 = meters_to_inches(y1)

ax.imshow(
    image, 
    aspect='auto',
    extent=(x0, x1, y0, y1),
    alpha=1.0,
    zorder=-1,
    origin='lower',
)
ax.set_aspect('equal', 'box')
ax.plot([0.0], [0.0], marker='x', color='k', markersize=10)

plot_grid_zone(ax, blue_grid_zone_string, 'b')
plot_grid_zone(ax, red_grid_zone_string, 'r')
plot_object(ax, transform_string)
plt.show()
