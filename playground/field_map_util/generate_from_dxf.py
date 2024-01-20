import argparse

import cv2
import ezdxf
import numpy as np
from tj2_tools.occupancy_grid import OccupancyGridManager
from tj2_tools.robot_state import Pose2d


def inch_to_meters(inch):
    return inch * 0.0254


def meter_to_inches(meter):
    return meter * 39.37


def mouse_callback(event, x, y, flags, param):
    ogm: OccupancyGridManager = param["ogm"]
    session = param["session"]
    if event == cv2.EVENT_LBUTTONDOWN:
        mask = np.zeros((ogm.height + 2, ogm.width + 2), np.uint8)
        grid_data = ogm.grid_data.astype(np.uint8)
        if session.mode == "unknown":
            new_value = 255
        elif session.mode == "wall":
            new_value = 100
        elif session.mode == "free":
            new_value = 0
        cv2.floodFill(grid_data, mask, seedPoint=(x, ogm.height - y), newVal=new_value)
        ogm.grid_data = grid_data.astype(np.int8)
        cv2.imshow(session.window_name, ogm.to_debug_image())


class DrawingSession:
    def __init__(self, window_name, mode):
        self.window_name = window_name
        self.mode = mode


def load_dxf_points(dxf_path: str) -> list:
    points = []
    doc = ezdxf.readfile(dxf_path)

    msp = doc.modelspace()
    lines = msp.query("LINE")
    for line in lines:
        start = line.get_dxf_attrib("start")
        end = line.get_dxf_attrib("end")

        x0 = inch_to_meters(start.x)
        y0 = inch_to_meters(start.y)
        x1 = inch_to_meters(end.x)
        y1 = inch_to_meters(end.y)

        points.append((x0, y0, x1, y1))
    points.append(points[0])

    return points


def main():
    parser = argparse.ArgumentParser(description="generate map from dxf", add_help=True)
    parser.add_argument("dxf", help="DXF file path")
    parser.add_argument("-o", "--output", default="map.yaml", help="Output path. Example: map.yaml")
    parser.add_argument("-l", "--line", default=1, help="line width")
    args = parser.parse_args()

    points = np.array(load_dxf_points(args.dxf))
    points[:, 1] *= -1
    points[:, 3] *= -1
    min_x = np.min(np.append(points[:, 0], points[:, 2]))
    min_y = np.min(np.append(points[:, 1], points[:, 3]))
    max_x = np.max(np.append(points[:, 0], points[:, 2]))
    max_y = np.max(np.append(points[:, 1], points[:, 3]))
    points[:, 0] -= min_x
    points[:, 2] -= min_x
    points[:, 1] -= min_y
    points[:, 3] -= min_y
    out_path = args.output
    ogm = OccupancyGridManager()

    resolution = 0.05  # meters/pixel
    border = 0
    width_meters = max_x - min_x
    height_meters = max_y - min_y
    width_px = int(np.ceil(width_meters / resolution)) + border * 2
    height_px = int(np.ceil(height_meters / resolution)) + border * 2

    points += border * resolution

    ogm.set_resolution(resolution)
    ogm.set_width(width_px)
    ogm.set_height(height_px)
    ogm.set_origin((0.0, 0.0))
    ogm.set_grid_data(np.zeros((height_px, width_px), dtype=np.int8))

    for x0, y0, x1, y1 in points:
        grid_x0, grid_y0 = ogm.get_costmap_x_y(x0, y0)
        grid_x1, grid_y1 = ogm.get_costmap_x_y(x1, y1)

        grid_y0 -= 1
        grid_y1 -= 1

        cv2.line(ogm.grid_data, (grid_x0, grid_y0), (grid_x1, grid_y1), (100,), int(args.line))

    show_image = ogm.to_debug_image()

    session = DrawingSession("map", "unknown")

    cv2.namedWindow(session.window_name)
    cv2.setMouseCallback(session.window_name, mouse_callback, param=dict(ogm=ogm, session=session))
    cv2.imshow(session.window_name, show_image)
    session_mode = session.mode
    while True:
        key = chr(cv2.waitKey(-1) & 0xFF)
        if key == "q":
            break
        elif key == "u":
            session_mode = "unknown"
        elif key == "w":
            session_mode = "wall"
        elif key == "f":
            session_mode = "free"
        elif key == "s":
            ogm.write_map(out_path)
            print("Saving to %s" % out_path)
        if session_mode != session.mode:
            session.mode = session_mode
            print(f"Fill mode: {session.mode}")
    ogm.write_map(out_path)
    print("Saving to %s" % out_path)


if __name__ == "__main__":
    main()
