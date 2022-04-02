import cv2
import sys
import ezdxf
import argparse
import numpy as np

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.occupancy_grid import OccupancyGridManager

def inch_to_meters(inch):
    return inch * 0.0254


def meter_to_inches(meter):
    return meter * 39.37


def mouse_callback(event, x, y, flags, param):
    ogm = param["ogm"]
    session = param["session"]
    if (event == cv2.EVENT_LBUTTONDOWN):
        mask = np.zeros((ogm.height + 2, ogm.width + 2), np.uint8)
        grid_data = ogm.grid_data.astype(np.uint8)
        if session.mode == "unknown":
            new_value = 255
        elif session.mode == "wall":
            new_value = 100
        elif session.mode == "free":
            new_value = 0
        cv2.floodFill(grid_data, mask, seedPoint=(x, y), newVal=new_value)
        ogm.grid_data = grid_data.astype(np.int8)
        cv2.imshow(session.window_name, ogm.get_image())


class DrawingSession:
    def __init__(self, window_name, mode):
        self.window_name = window_name
        self.mode = mode


def main():
    parser = argparse.ArgumentParser(description="generate map from dxf", add_help=True)
    parser.add_argument("dxf", help="DXF file path")
    parser.add_argument("-o", "--output", default="map.yaml", help="Output path")
    parser.add_argument("-l", "--line", default=1, help="line width")
    args = parser.parse_args()

    out_path = args.output
    ogm = OccupancyGridManager()

    resolution = 0.05  # meters/pixel
    width_meters = 20.0
    height_meters = 20.0
    width_px = int(width_meters / resolution)
    height_px = int(height_meters / resolution)

    ogm.set_resolution(resolution)
    ogm.set_width(width_px)
    ogm.set_height(height_px)
    ogm.set_origin((-width_meters / 2, -height_meters / 2))
    # ogm.set_origin((0.0, 0.0))
    ogm.set_image(np.zeros((height_px, width_px), dtype=np.int8))

    doc = ezdxf.readfile(args.dxf)

    msp = doc.modelspace()
    lines = msp.query("LINE")
    for line in lines:
        start = line.get_dxf_attrib("start")
        end = line.get_dxf_attrib("end")

        x0 = inch_to_meters(start.x)
        y0 = inch_to_meters(start.y)
        x1 = inch_to_meters(end.x)
        y1 = inch_to_meters(end.y)

        grid_pt1 = ogm.get_costmap_x_y(x0, y0)
        grid_pt2 = ogm.get_costmap_x_y(x1, y1)

        cv2.line(ogm.grid_data, grid_pt1, grid_pt2, (100,), int(args.line))

    show_image = ogm.get_image()

    session = DrawingSession("map", "unknown")

    cv2.namedWindow(session.window_name)
    cv2.setMouseCallback(session.window_name, mouse_callback, param=dict(ogm=ogm, session=session))
    cv2.imshow(session.window_name, show_image)
    while True:
        key = chr(cv2.waitKey(-1) & 0xff)
        if key == 'q':
            break
        elif key == 'u':
            session.mode = "unknown"
        elif key == 'w':
            session.mode = "wall"
        elif key == 'f':
            session.mode = "free"
        elif key == 's':
            ogm.to_file(out_path)
            print("Saving to %s" % out_path)
    ogm.to_file(out_path)
    print("Saving to %s" % out_path)

if __name__ == "__main__":
    main()
