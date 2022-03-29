import os
import cv2
import csv
import sys
import math
import rospkg
import numpy as np

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.robot_state import Pose2d
from tj2_tools.occupancy_grid import OccupancyGridManager

from file_management import load_waypoints

# map_name = "br-114-03-26-2022"
map_name = "rapid-react-2022-02-19T07-55-53--407688-edited"

rospack = rospkg.RosPack()
in_map_path = os.path.join(rospack.get_path("tj2_laser_slam"), "maps", map_name + ".yaml")
out_map_path = os.path.join(rospack.get_path("tj2_turret"), "maps", map_name + ".yaml")
data_path = os.path.join(rospack.get_path("tj2_turret"), "config", "recorded_data.csv")

ogm = OccupancyGridManager.from_cost_file(in_map_path)
waypoints = load_waypoints(map_name)
center = waypoints["center"]

turret_base_link_x_offset = -0.050456

probabilities = []
with open(data_path) as file:
    reader = csv.reader(file)
    header = next(reader)
    for row in reader:
        if row[0].strip().startswith("#"):
            continue
        if row[0] != "prob":
            continue
        data = {
            "probability": float(row[header.index("value")]),
            "distance": float(row[header.index("distance")])
        }
        hood = row[header.index("hood")]
        # if hood == "up":
        #     continue
        print("%s\t%0.2f\t%0.3f" % (hood, data["probability"], data["distance"]))
        probabilities.append(data)

field_map = ogm.get_image()

ogm.grid_data[:] = 100
probabilities.sort(key=lambda x: x["distance"], reverse=True)

robot_radius = 0.5 / ogm.resolution

for row in probabilities:
    distance = row["distance"]
    x = center.x + distance + turret_base_link_x_offset
    y = center.y + distance + turret_base_link_x_offset
    image_x, image_y = ogm.get_costmap_x_y(x, y)
    
    origin = ogm.get_costmap_x_y(center.x, center.y)

    x = image_x - origin[0]
    y = image_y - origin[1]
    radius = int(math.sqrt(x * x + y * y))
    radius -= int(robot_radius)
    if radius < 0:
        radius = 1
    probability = 1.0 - row["probability"]
    
    cv2.circle(ogm.grid_data, origin, radius, (probability * 100,), -1)
    # cv2.circle(ogm.grid_data, (image_x, image_y), int(robot_radius), (probability * 100,), -1)

show_image = cv2.addWeighted(field_map, 0.5, ogm.get_image(), 0.5, 0.0)

ogm.to_file(out_map_path)

cv2.imshow("map", show_image)
while True:
    key = chr(cv2.waitKey(-1) & 0xff)
    if key == 'q':
        break
