import argparse
import copy
import os

import numpy as np
from sensor_msgs.msg import CameraInfo
from tj2_tools.camera_calibration.calibration_file import read_yaml_parameters, write_yaml_parameters


def resize_camera_info(info: CameraInfo, destination_width: int, destination_height: int) -> CameraInfo:
    info = copy.deepcopy(info)
    scale_y = destination_height / info.height
    scale_x = destination_width / info.width
    info.height = destination_height
    info.width = destination_width

    k = np.array(info.K)
    p = np.array(info.P)

    k[0] *= scale_x  # fx
    k[2] *= scale_x  # cx
    k[4] *= scale_y  # fy
    k[5] *= scale_y  # cy

    p[0] *= scale_x  # fx
    p[2] *= scale_x  # cx
    p[3] *= scale_x  # T
    p[5] *= scale_y  # fy
    p[6] *= scale_y  # cy

    info.roi.x_offset = int(info.roi.x_offset * scale_x)
    info.roi.y_offset = int(info.roi.y_offset * scale_y)
    info.roi.width = int(info.roi.width * scale_x)
    info.roi.height = int(info.roi.height * scale_y)

    info.K = k.tolist()
    info.P = p.tolist()

    return info


def main() -> None:
    parser = argparse.ArgumentParser("scale_camera_info")
    parser.add_argument(
        "path",
        type=str,
        help="path to camera info yaml file. "
        "ex: /opt/tj2/tj2_ros/src/tj2_bringup/config/shared/arducam/calibration/TJ2003_1600x1200.yaml",
    )
    parser.add_argument("width", type=int, help="Desired width")
    parser.add_argument("height", type=int, help="Desired height")
    parser.add_argument("out_name", type=str, help="Output file name. ex: TJ2003_800x600.yaml")
    args = parser.parse_args()

    path = args.path
    out_dir = os.path.dirname(path)
    out_path = os.path.join(out_dir, args.out_name)

    info, name = read_yaml_parameters(path)
    resized_info = resize_camera_info(info, args.width, args.height)
    out_path = out_path.replace("/home/tj2/ros_ws/src/tj2_ros", "/home/tj2/tj2_ros/src")
    write_yaml_parameters(out_path, resized_info, name)
    print(f"Saved camera info to {out_path}")


if __name__ == "__main__":
    main()
