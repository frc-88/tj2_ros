import cv2
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.pyplot import Axes
import numpy as np
from tj2_tools.occupancy_grid import OccupancyGridManager
from tj2_charged_up.grid_zone_manager import GridZoneManager, Alliance

matplotlib.use('TkAgg')

def plot_grid_zone(ax: Axes, grid_zones: GridZoneManager):
    red_points = []
    blue_points = []
    for zone in grid_zones.zones:
        if zone.alliance == Alliance.RED:
            points = red_points
        else:
            points = blue_points
        
        points.append([zone.x, zone.y, zone.z])
        # ax.annotate(f"{zone.row}-{zone.column}", (zone.x, zone.y))
    red_points = np.array(red_points)
    blue_points = np.array(blue_points)
    ax.scatter(red_points[:, 0], red_points[:, 1], color='red')
    ax.scatter(blue_points[:, 0], blue_points[:, 1], color='blue')


def plot_map(ax: Axes, ogm: OccupancyGridManager):
    image = ogm.to_debug_image()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    x0, y0 = ogm.get_world_x_y(0, ogm.height)
    x1, y1 = ogm.get_world_x_y(ogm.width, 0)
    
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

fig = plt.figure(1)
ax = fig.add_subplot()

ogm = OccupancyGridManager.from_map_file("/root/tj2_ros/src/tj2_data/data/maps/charged-up-2023.yaml")
grid_zones = GridZoneManager.from_file("../config/grid_zones.csv")

plot_grid_zone(ax, grid_zones)
plot_map(ax, ogm)

plt.show()