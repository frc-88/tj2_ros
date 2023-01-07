#!/usr/bin/env python3

import cv2
import numpy as np
import os
import glob
import yaml
import time

# Defining the dimensions of checkerboard
CHECKERBOARD = (5, 8)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 28, 0.001)

# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = [] 


# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

show_images = True

# Extracting path of individual image stored in a given directory
images = glob.glob('./images/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    
    # If desired number of corner are detected,
    # we refine the pixel coordinates and display 
    # them on the images of checker board

    if ret == True:
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
    
    if show_images:
        cv2.imshow('img',img)
        key = chr(cv2.waitKey(15) & 0xff)
        if key == 'q':
            quit()

if show_images:
    cv2.destroyAllWindows()

h,w = img.shape[:2]


# Performing camera calibration by 
# passing the value of known 3D points (objpoints)
# and corresponding pixel coordinates of the 
# detected corners (imgpoints)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
proj_mtx = np.append(mtx, np.zeros((3, 1)), axis=1)

print("Camera matrix : \n")
print(mtx)
print("dist : \n")
print(dist)
# print("rvecs : \n")
# print(rvecs)
# print("tvecs : \n")
# print(tvecs)
print("proj : \n")
print(proj_mtx)


assert mtx.shape == (3, 3)
assert dist.shape == (1, 5)

config = dict(
    image_width=320,
    image_height=240,
    camera_name="limelight",
    camera_matrix=dict(
        rows=3,
        cols=3,
        data=mtx.flatten().tolist()
    ),
    distortion_model="plumb_bob",
    distortion_coefficients=dict(
        rows=1,
        cols=5,
        data=dist.flatten().tolist()
    ),
    rectification_matrix=dict(
        rows=3,
        cols=3,
        data=np.eye(3).flatten().tolist()
    ),
    projection_matrix=dict(
        rows=3,
        cols=4,
        data=proj_mtx.flatten().tolist()
    )
)

with open("320x240.yaml", 'w') as file:
    yaml.dump(config, file)
