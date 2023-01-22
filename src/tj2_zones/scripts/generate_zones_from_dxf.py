import os
import math
import numpy as np
from typing import Optional
import cv2
import yaml
import ezdxf
import argparse
import shapely.geometry.polygon
from shapely.geometry import Polygon

from tj2_tools.zone import ZoneManager
from tj2_tools.occupancy_grid import OccupancyGridManager

def inch_to_meters(inch):
    return inch * 0.0254


def meter_to_inches(meter):
    return meter * 39.37


class DrawingSession:
    def __init__(self, window_name, mode):
        self.window_name = window_name
        self.mode = mode

def to_radial(point, origin):
    vector = np.array([
        point[0] - origin[0],
        point[1] - origin[1]
    ])
    radius = np.linalg.norm(vector)
    if radius == 0.0:
        return -np.pi, 0.0
    angle = np.arctan2(vector[1], vector[0])
    if angle < 0.0:
        angle += 2 * math.pi
    return angle, radius

def load_config(path):
    with open(path) as file:
        return yaml.safe_load(file)

def main():
    parser = argparse.ArgumentParser(description="generate zones from dxfs", add_help=True)
    parser.add_argument("dxfs", type=str, default=".",
                        help="DXF directory to convert to zones")
    parser.add_argument("-o", "--output", default="zones.bin", help="Output path")
    parser.add_argument("-m", "--map", default="", help="Map to draw zones on for debugging (ex. map.yaml)")
    args = parser.parse_args()

    out_path = args.output
    manager = ZoneManager("map")
    
    source_paths = []
    config = {}
    for dirpath, dirnames, filenames in os.walk(args.dxfs):
        for filename in filenames:
            path = os.path.join(dirpath, filename)
            if filename.endswith(".dxf"):
                source_paths.append(path)
            if filename.endswith(".yaml"):
                config = load_config(path)
    assert len(source_paths) > 0

    names = []
    max_x: Optional[float] = None
    min_x: Optional[float] = None
    max_y: Optional[float] = None
    min_y: Optional[float] = None
    for path in source_paths:
        name = os.path.splitext(os.path.basename(path))[0]
        name = name.replace(" ", "_").replace("-", "_")

        doc = ezdxf.readfile(path)
        
        points = []
        
        msp = doc.modelspace()
        lines = msp.query("LINE")
        for line in lines:
            start = line.get_dxf_attrib("start")
            end = line.get_dxf_attrib("end")

            x0 = inch_to_meters(start.x)
            y0 = inch_to_meters(start.y)
            x1 = inch_to_meters(end.x)
            y1 = inch_to_meters(end.y)
            points.append((x0, y0))
            points.append((x1, y1))
            
            check_max_x = max(x0, x1)
            check_max_y = max(y0, y1)
            check_min_x = min(x0, x1)
            check_min_y = min(y0, y1)
            max_x = check_max_x if max_x is None or max_x < check_max_x else max_x
            min_x = check_min_x if min_x is None or min_x > check_min_x else min_x
            max_y = check_max_y if max_y is None or max_y < check_max_y else max_y
            min_y = check_min_y if min_y is None or min_y > check_min_y else min_y
        origin = np.median(points, axis=0)
        if name in config:
            priority = config[name].get("priority", 0.0)
            origin_offset = config[name].get("origin_offset", np.array([0.0, 0.0]))
            sort_points = config[name].get("sort_points", False)
        else:
            priority = 0.0
            origin_offset = np.array([0.0, 0.0])
            sort_points = False
        if sort_points:
            origin += origin_offset
            points.sort(key=lambda point: to_radial(point, origin))
        polygon = Polygon(points)
        manager.add_polygon(name, priority, polygon)
        names.append(name)
    
    print("Saving to %s" % out_path)
    manager.save_zones(out_path)
    
    if len(args.map) > 0:
        ogm = OccupancyGridManager.from_map_file(args.map)
        overlay = True
    else:
        assert max_x is not None
        assert min_x is not None
        assert max_y is not None
        assert min_y is not None
        ogm = OccupancyGridManager()
        resolution = 0.05
        ogm.set_resolution(resolution)
        width = (max_x - min_x) / resolution
        height = (max_y - min_y) / resolution
        ogm.set_width(int(width))
        ogm.set_height(int(height))
        ogm.set_origin((min_x, min_y))
        overlay = False
    ogm.set_reference_frame("map")
    
    window_name = "zones"
    cv2.namedWindow(window_name)
    current_nogo = 0
    manager.update_nogos([names[current_nogo]])
    while True:
        grid_data = manager.to_grid_data(ogm, overlay_base=overlay)
        new_ogm = OccupancyGridManager.from_ogm(ogm)
        new_ogm.set_grid_data(grid_data)
        cv2.imshow(window_name, new_ogm.to_debug_image())
        press = cv2.waitKey(-1)
        key = chr(press & 0xff)
        if key == 'q':
            break
        elif press == 81:  # left
            current_nogo -= 1
        elif press == 83:  # right
            current_nogo += 1
        
        current_nogo = min(len(names), max(0, current_nogo))
    
        if current_nogo == len(names):
            manager.update_nogos([])
        else:
            manager.update_nogos([names[current_nogo]])
        

if __name__ == "__main__":
    main()
