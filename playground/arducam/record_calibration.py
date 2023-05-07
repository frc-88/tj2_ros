#!/usr/bin/env python3
import os
import sys
import threading
import cv2
import rospy
import argparse
import numpy as np
from dataclasses import dataclass, field
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


@dataclass
class AppData:
    window_name: str
    bridge: CvBridge
    lock: threading.Lock
    frame: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.uint8))


def image_callback(data: AppData, message: Image) -> None:
    bridge = data.bridge
    frame = bridge.imgmsg_to_cv2(message, "passthrough")

    with data.lock:
        data.frame = frame


def adjust_gamma(image, gamma=1.0):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values
    invGamma = 1.0 / gamma
    table = np.array(
        [((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]
    ).astype("uint8")
    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)


def adjust_brightness(image, brightness):
    brightness = max(min(brightness, 255), -255)
    if brightness > 0:
        shadow = brightness
        highlight = 255
    else:
        shadow = 0
        highlight = 255 + brightness

    alpha = (highlight - shadow) / 255
    gamma = shadow

    # The function addWeighted calculates the weighted sum of two arrays
    return cv2.addWeighted(image, alpha, image, 0, gamma)


def adjust_contrast(image, contrast):
    contrast = max(min(contrast, 127), -127)
    alpha = float(131 * (contrast + 127)) / (127 * (131 - contrast))
    gamma = 127 * (1 - alpha)

    # The function addWeighted calculates
    # the weighted sum of two arrays
    return cv2.addWeighted(image, alpha, image, 0, gamma)


def main():
    parser = argparse.ArgumentParser("record_calibration")
    parser.add_argument("camera_num", type=int)
    parser.add_argument("directory", type=str)
    parser.add_argument("board_width", type=int)
    parser.add_argument("board_height", type=int)
    args = parser.parse_args()

    camera_num = args.camera_num
    write_directory = args.directory
    checkerboard = (args.board_width, args.board_height)
    window_name = "calibration"

    if not os.path.isdir(write_directory):
        os.makedirs(write_directory)
        print("Making directory:", write_directory)

    data = AppData(window_name, CvBridge(), threading.Lock())

    cv2.namedWindow(window_name)

    rospy.init_node("record_calibration")
    subscriber = rospy.Subscriber(
        f"camera_{camera_num}/image_raw",
        Image,
        lambda msg: image_callback(data, msg),
        queue_size=1,
    )
    image_count = 0

    try:
        while True:
            with data.lock:
                frame = data.frame
                if len(frame) == 0:
                    frame = np.zeros((300, 300), np.uint8)
            if rospy.is_shutdown():
                break
            # frame = adjust_brightness(frame, 80)
            # frame = adjust_gamma(frame, 1.5)
            # frame = adjust_contrast(frame, 30)
            success, corners = cv2.findChessboardCorners(
                frame,
                checkerboard,
                cv2.CALIB_CB_ADAPTIVE_THRESH
                + cv2.CALIB_CB_FAST_CHECK
                + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            # print(f"Found checkerboard: {success}, {corners}")
            debug = cv2.drawChessboardCorners(
                np.copy(frame), checkerboard, corners, success
            )

            cv2.imshow(window_name, debug)
            key = chr(cv2.waitKey(1) & 0xFF)
            if key == "q":
                break
            elif key == "s":
                name = f"{image_count:06d}.jpg"
                path = os.path.join(write_directory, name)
                print(f"Writing to {path}")
                cv2.imwrite(path, frame)
                assert os.path.isfile(path), path
                image_count += 1
    finally:
        subscriber.unregister()


if __name__ == "__main__":
    main()
