import sys
import cv2
import numpy as np

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.occupancy_grid import OccupancyGridManager


def test_map():
    maps = [
        "rapid-react-2022-02-19T07-55-53--407688",
        "rapid-react-2022",
        "3-101"
    ]
    cv2.namedWindow('map')
    for name in maps:
        ogm = OccupancyGridManager.from_map_file("maps/" + name + ".yaml")
        ogm.write_map("maps/test-map.yaml")
        test_ogm = OccupancyGridManager.from_map_file("maps/test-map.yaml")
        assert np.all(test_ogm.grid_data == ogm.grid_data)
        cv2.imshow('map', test_ogm.to_debug_image())
        cv2.waitKey(-1)

def test_costmap():
    maps = [
        "probability-1",
        "probability-2",
        "probability-3",
    ]
    cv2.namedWindow('map')
    for name in maps:
        ogm = OccupancyGridManager.from_costmap_file("maps/" + name + ".yaml")
        ogm.write_costmap("maps/test-costmap.yaml")
        test_ogm = OccupancyGridManager.from_costmap_file("maps/test-costmap.yaml")
        assert np.all(test_ogm.grid_data == ogm.grid_data)
        cv2.imshow('map', test_ogm.to_debug_image())
        cv2.waitKey(-1)
