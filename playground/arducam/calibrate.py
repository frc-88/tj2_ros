#!/usr/bin/env python3
import cv2
import glob
import argparse
import numpy as np


def main():
    parser = argparse.ArgumentParser("calibrate")
    parser.add_argument("directory", type=str)
    parser.add_argument("board_width", type=int)
    parser.add_argument("board_height", type=int)
    args = parser.parse_args()

    read_directory = args.directory

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

    gray = None
    images = glob.glob(f"./{read_directory}/*.jpg")
    for filename in images:
        image = cv2.imread(filename)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
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
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None
    )
    print("Camera matrix : \n")
    print(mtx)
    print("dist : \n")
    print(dist)
    print("rvecs : \n")
    print(rvecs)
    print("tvecs : \n")
    print(tvecs)


if __name__ == "__main__":
    main()
