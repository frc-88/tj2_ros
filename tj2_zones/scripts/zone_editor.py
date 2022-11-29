import os
import sys
import cv2
import numpy as np

import rospkg

from geometry_msgs.msg import Point

from tj2_interfaces.msg import Zone
from tj2_interfaces.msg import ZoneArray

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.zone import ZoneManager
from tj2_tools.occupancy_grid import OccupancyGridManager

rospack = rospkg.RosPack()

frame_id = "map"
field_map_name = "rapid-react-2022-02-19T07-55-53--407688"
# field_map_name = "rapid-react-2022"

zones = ZoneArray()
zones.header.frame_id = frame_id

field_x = 8.230
field_y = 4.115

divide_red_x = 1.832
divide_red_y = -4.115

divide_blue_x = -1.832
divide_blue_y = 4.115

zone1 = Zone()
zone1.header.frame_id = frame_id
zone1.name = "red"
zone1.priority = 0.0
zone1.points.append(Point(x=divide_red_x, y=divide_red_y))
zone1.points.append(Point(x=field_x, y=-field_y))
zone1.points.append(Point(x=field_x, y=field_y))
zone1.points.append(Point(x=divide_blue_x, y=divide_blue_y))

zone2 = Zone()
zone2.header.frame_id = frame_id
zone2.name = "blue"
zone2.priority = 0.0
zone2.points.append(Point(x=divide_blue_x, y=divide_blue_y))
zone2.points.append(Point(x=-field_x, y=field_y))
zone2.points.append(Point(x=-field_x, y=-field_y))
zone2.points.append(Point(x=divide_red_x, y=divide_red_y))

zones.zones.append(zone1)
zones.zones.append(zone2)

window_name = "zones"

names = ["red", "blue"]

manager = ZoneManager.from_msg(zones)

field_map_path = os.path.join(rospack.get_path("tj2_laser_slam"), "maps", field_map_name + ".yaml")
# ogm = OccupancyGridManager.from_cost_file(field_map_path)
ogm = OccupancyGridManager.from_map_file(field_map_path)

# ogm.to_file("some-map.yaml")

# print(np.min(ogm.grid_data), np.max(ogm.grid_data))

current_nogo = 0

cv2.namedWindow(window_name)
while True:
    grid_data = manager.to_image(ogm)
    new_ogm = OccupancyGridManager.from_ogm(ogm)
    new_ogm.set_grid_data(grid_data)
    cv2.imshow(window_name, new_ogm.get_image())
    key = chr(cv2.waitKey(-1) & 0xff)
    if key == 'q':
        break
    if current_nogo < 0:
        manager.update_nogos([])
    else:
        manager.update_nogos([names[current_nogo]])
    current_nogo += 1
    if current_nogo >= len(names):
        current_nogo = -1

manager.save_zones(field_map_name + ".bin")
