from typing import Tuple

import yaml
from sensor_msgs.msg import CameraInfo


def write_yaml_parameters(path: str, info: CameraInfo, camera_name: str):
    with open(path, "w") as file:
        image_height = info.height
        image_width = info.width
        distortion_model = info.distortion_model
        distortion_coefficients = info.D
        camera_matrix = info.K
        rectification_matrix = info.R
        projection_matrix = info.P
        contents = f"""
image_width: {image_width}
image_height: {image_height}
camera_name: {camera_name}
camera_matrix:
  rows: 3
  cols: 3
  data: {camera_matrix}
distortion_model: {distortion_model}
distortion_coefficients:
  rows: 1
  cols: 5
  data: {distortion_coefficients}
rectification_matrix:
  rows: 3
  cols: 3
  data: {rectification_matrix}
projection_matrix:
  rows: 3
  cols: 4
  data: {projection_matrix}
"""
        file.write(contents)


def read_yaml_parameters(path: str) -> Tuple[CameraInfo, str]:
    with open(path) as file:
        config = yaml.safe_load(file)
    info = CameraInfo()
    info.width = config["image_width"]
    info.height = config["image_height"]
    camera_name = config["camera_name"]
    info.distortion_model = config["distortion_model"]
    info.K = config["camera_matrix"]["data"]
    info.D = config["distortion_coefficients"]["data"]
    info.R = config["rectification_matrix"]["data"]
    info.P = config["projection_matrix"]["data"]
    return info, camera_name
