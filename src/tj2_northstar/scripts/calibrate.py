#!/usr/bin/env python3
import os
import cv2
import yaml
import glob
import argparse
import numpy as np


def adjust_gamma(image, gamma=1.0):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values
    invGamma = 1.0 / gamma
    table = np.array(
        [((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]
    ).astype("uint8")
    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)


def write_parameters(path, parameters):
    print(f"Wrote to {path}")
    with open(path, "w") as file:
        yaml.dump(parameters, file)


def main():
    parser = argparse.ArgumentParser("calibrate")
    parser.add_argument("directory", type=str)
    parser.add_argument(
        "board_width", type=int, help="Number of internal width corners"
    )
    parser.add_argument(
        "board_height", type=int, help="Number of internal height corners"
    )
    parser.add_argument("square_size", type=float, help="Size of square in meters")
    args = parser.parse_args()

    read_directory = args.directory
    square_size = args.square_size

    # Define the dimensions of checkerboard
    checkerboard = (args.board_width, args.board_height)

    # stop the iteration when specified
    # accuracy, epsilon, is reached or
    # specified number of iterations are completed.
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

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
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if success:
            obj_points.append(objp)
            # refining pixel coordinates for given 2d points.
            refined_corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )

            img_points.append(refined_corners)
        else:
            print(f"Failed to find checkerboard in {filename}")

    assert gray is not None
    assert shape is not None
    success, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None
    )

    if success:
        distortion_coeffs = dist[0].tolist()
        camera_matrix = mtx.flatten().tolist()
        rectification_matrix = np.eye(3).flatten().tolist()
        projection = np.zeros((3, 4))
        projection[0:3, 0:3] = mtx
        projection_matrix = projection.flatten().tolist()

        parameters = {
            "height": shape[0],
            "width": shape[1],
            "distortion_model": "plumb_bob",
            "D": distortion_coeffs,
            "K": camera_matrix,
            "R": rectification_matrix,
            "P": projection_matrix,
        }
        write_parameters(os.path.join(read_directory, "camera.yaml"), parameters)

    else:
        print("Failed to compute camera parameters!")


if __name__ == "__main__":
    main()
