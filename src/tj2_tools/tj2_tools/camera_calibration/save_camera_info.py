import os

from camera_info_manager import (
    CameraInfoMissingError,  # type: ignore
    URL_file,  # type: ignore
    URL_package,  # type: ignore
    getPackageFileName,  # type: ignore
    parseURL,  # type: ignore
    resolveURL,  # type: ignore
)
from sensor_msgs.msg import CameraInfo


def write_yaml_parameters(path: str, info: CameraInfo):
    with open(path, "w") as file:
        image_height = info.height
        image_width = info.width
        distortion_model = info.distortion_model
        distortion_coefficients = info.D
        camera_matrix = info.K
        rectification_matrix = info.R
        projection_matrix = info.P
        camera_name = info.header.frame_id
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


def resolve_url(url: str, camera_name: str) -> str:
    resolved_url = resolveURL(url, camera_name)
    url_type = parseURL(resolved_url)
    if url_type == URL_file or os.path.isfile(resolved_url):
        return resolved_url
    elif url_type == URL_package:
        filename = getPackageFileName(resolved_url)
        if filename == "":  # package not resolved
            raise CameraInfoMissingError("Calibration package missing.")
        return filename
    else:
        return resolved_url


def save_camera_info(out_url: str, info: CameraInfo) -> None:
    resolved_url = resolve_url(out_url, info.header.frame_id)
    write_yaml_parameters(resolved_url, info)
