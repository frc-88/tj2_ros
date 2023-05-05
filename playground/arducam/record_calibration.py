import sys
from typing import Tuple
import threading
import cv2
import rospy
import argparse
import numpy as np
from dataclasses import dataclass, field
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


@dataclass
class AppData:
    window_name: str
    checkboard: Tuple[int, int]
    bridge: CvBridge
    lock: threading.Lock
    frame: np.ndarray = field(default_factory=np.array([]))


def image_callback(config: AppData, message: Image) -> None:
    checkerboard = config.checkerboard
    window_name = config.window_name
    bridge = config.bridge

    success, corners = cv2.findChessboardCorners(
        frame,
        checkerboard,
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_FAST_CHECK
        + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    frame = cv2.drawChessboardCorners(frame, checkerboard, corners, success)
    config.frame = frame


def main():
    parser = argparse.ArgumentParser("record_calibration")
    parser.add_argument("camera-num", type=int)
    parser.add_argument("directory", type=str)
    parser.add_argument("board-width", type=int)
    parser.add_argument("board-height", type=int)
    args = parser.parse_args()

    camera_num = args.camera_num
    checkerboard = (args.board_width, args.board_height)
    window_name = "calibration"

    data = AppData(window_name, checkerboard, CvBridge(), threading.Lock())

    cv2.namedWindow(window_name)

    rospy.init_node("record_calibration")
    subscriber = rospy.Subscriber(
        f"camera_{camera_num}/image_rect",
        Image,
        lambda msg: image_callback(data, msg),
    )

    while True:
        data.frame
        cv2.imshow(window_name, frame)
        key = chr(cv2.waitKey(1) & 0xFF)
        if key == "q":
            sys.exit(0)
