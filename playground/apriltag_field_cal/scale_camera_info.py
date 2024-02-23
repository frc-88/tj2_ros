import argparse
import copy
import os

import numpy as np
from camera_info_manager import CameraInfoManager  # type: ignore
from sensor_msgs.msg import CameraInfo
from tj2_tools.camera_calibration.save_camera_info import resolve_url, save_camera_info


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


def load_camera_info(url: str) -> CameraInfo:
    info_manager = CameraInfoManager("camera", url=url, namespace="/")
    info_manager.loadCameraInfo()
    return info_manager.getCameraInfo()


def main() -> None:
    parser = argparse.ArgumentParser("scale_camera_info")
    parser.add_argument(
        "url",
        type=str,
        help="URL to camera info yaml file. "
        "ex: package://tj2_bringup/config/shared/arducam/calibration/TJ2003_1600x1200.yaml",
    )
    parser.add_argument("width", type=int, help="Desired width")
    parser.add_argument("height", type=int, help="Desired height")
    parser.add_argument("out_name", type=str, help="Output file name. ex: TJ2003_800x600.yaml")
    args = parser.parse_args()

    url = args.url
    path = resolve_url(url, "camera")
    out_dir = os.path.dirname(path)
    out_path = os.path.join(out_dir, args.out_name)

    info = load_camera_info(url)
    resized_info = resize_camera_info(info, args.width, args.height)
    out_path = out_path.replace("/home/tj2/ros_ws/src/tj2_ros", "/home/tj2/tj2_ros/src")
    save_camera_info(out_path, resized_info)
    print(f"Saved camera info to {out_path}")


if __name__ == "__main__":
    main()
