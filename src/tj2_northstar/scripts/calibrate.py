#!/usr/bin/env python3
import argparse
import glob
import os

import cv2
import numpy as np
from camera_info_manager import CameraInfoManager  # type: ignore


def write_yaml_parameters(path, parameters):
    print(f"Wrote to {path}")
    with open(path, "w") as file:
        image_height = parameters["image_height"]
        image_width = parameters["image_width"]
        distortion_model = parameters["distortion_model"]
        distortion_coefficients = parameters["distortion_coefficients"]
        camera_matrix = parameters["camera_matrix"]
        rectification_matrix = parameters["rectification_matrix"]
        projection_matrix = parameters["projection_matrix"]
        camera_name = parameters["camera_name"]
        contents = f"""
image_width: {image_width}
image_height: {image_height}
camera_name: {camera_name}
camera_matrix:
  rows: {camera_matrix["rows"]}
  cols: {camera_matrix["cols"]}
  data: {camera_matrix["data"]}
distortion_model: {distortion_model}
distortion_coefficients:
  rows: {distortion_coefficients["rows"]}
  cols: {distortion_coefficients["cols"]}
  data: {distortion_coefficients["data"]}
rectification_matrix:
  rows: {rectification_matrix["rows"]}
  cols: {rectification_matrix["cols"]}
  data: {rectification_matrix["data"]}
projection_matrix:
  rows: {projection_matrix["rows"]}
  cols: {projection_matrix["cols"]}
  data: {projection_matrix["data"]}
"""
        file.write(contents)


def get_matrix_dict(matrix):
    data = matrix.flatten().tolist()
    if len(matrix.shape) == 1:
        rows = 1
        cols = matrix.shape[0]
    else:
        rows = matrix.shape[0]
        cols = matrix.shape[1]
    return {
        "data": data,
        "rows": rows,
        "cols": cols,
    }


def main():
    parser = argparse.ArgumentParser("calibrate")
    parser.add_argument("directory", type=str)
    parser.add_argument("board_width", type=int, help="Number of internal width corners")
    parser.add_argument("board_height", type=int, help="Number of internal height corners")
    parser.add_argument("square_size", type=float, help="Size of square in meters")
    parser.add_argument("-g", "--guess", default="", type=str, help="Use another camera's parameters as a guess")
    parser.add_argument("-o", "--output", default="")
    parser.add_argument("-d", "--debug", action="store_true")
    args = parser.parse_args()

    print(f"Board size: {args.board_width}x{args.board_height}. Square size: {args.square_size} m")

    read_directory = args.directory
    camera_name = os.path.basename(read_directory)
    write_directory = os.path.dirname(read_directory)
    square_size = args.square_size

    # Define the dimensions of checkerboard
    checkerboard = (args.board_width, args.board_height)

    # stop the iteration when specified
    # accuracy, epsilon, is reached or
    # specified number of iterations are completed.
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

    # Creating vector to store vectors of 3D points for each checkerboard image
    obj_points = []

    # Creating vector to store vectors of 2D points for each checkerboard image
    img_points = []

    # Defining the world coordinates for 3D points
    objp = np.zeros((1, checkerboard[0] * checkerboard[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0 : checkerboard[0], 0 : checkerboard[1]].T.reshape(-1, 2)
    objp *= square_size

    gray = None
    shape = None
    images = glob.glob(f"./{read_directory}/*.jpg")
    for filename in images:
        image = cv2.imread(filename)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if shape is None:
            shape = gray.shape
        else:
            assert shape == gray.shape, "Images are not all the same size!"
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        success, corners = cv2.findChessboardCorners(
            gray,
            checkerboard,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE
            + cv2.CALIB_CB_FILTER_QUADS,
        )
        if success:
            cv2.drawChessboardCorners(image, checkerboard, corners, True)
            obj_points.append(objp)
            # refining pixel coordinates for given 2d points.
            refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            img_points.append(refined_corners)
        else:
            print(f"Failed to find checkerboard in {filename}")
        if args.debug:
            cv2.imshow("image", image)
            key = chr(cv2.waitKey(-1) & 0xFF)
            if key == "q":
                return

    assert gray is not None
    assert shape is not None
    if args.guess:
        guess_path = "file://" + os.path.abspath(args.guess)
        info_manager = CameraInfoManager("guess", guess_path)
        info_manager.loadCameraInfo()
        info = info_manager.getCameraInfo()
        camera_matrix_guess = np.array(info.K, dtype=np.float32).reshape(3, 3)
        flags = cv2.CALIB_USE_INTRINSIC_GUESS
    else:
        camera_matrix_guess = None
        flags = 0
    success, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], camera_matrix_guess, None, None, None, flags, criteria
    )

    if success:
        write_path = (
            os.path.join(write_directory, f"{camera_name}_{shape[1]}x{shape[0]}.yaml")
            if len(args.output) == 0
            else args.output
        )
        distortion_coeffs = dist[0]
        camera_matrix = mtx
        rectification_matrix = np.eye(3)
        projection = np.zeros((3, 4))
        projection[0:3, 0:3] = mtx
        projection_matrix = projection

        parameters = {
            "image_height": shape[0],
            "image_width": shape[1],
            "distortion_model": "plumb_bob",
            "distortion_coefficients": get_matrix_dict(distortion_coeffs),
            "camera_matrix": get_matrix_dict(camera_matrix),
            "rectification_matrix": get_matrix_dict(rectification_matrix),
            "projection_matrix": get_matrix_dict(projection_matrix),
            "camera_name": camera_name,
        }
        write_yaml_parameters(write_path, parameters)

    else:
        print("Failed to compute camera parameters!")


if __name__ == "__main__":
    main()
