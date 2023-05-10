#!/usr/bin/env python3
import os
import threading
from typing import Tuple
import cv2
import rospy
import argparse
from queue import Queue
import numpy as np
from dataclasses import dataclass, field
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


@dataclass
class AppData:
    window_name: str
    bridge: CvBridge
    lock: threading.Lock
    chessboard: Tuple[int, int]
    frame: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.uint8))


def image_callback(data: AppData, message: Image) -> None:
    bridge = data.bridge
    frame = bridge.imgmsg_to_cv2(message, "passthrough")

    with data.lock:
        data.frame = frame


def detection_task(data: AppData, in_queue: Queue, out_queue: Queue):
    chessboard = data.chessboard
    while not rospy.is_shutdown():
        frame = None
        while not in_queue.empty():
            frame = in_queue.get()
        if frame is None:
            rospy.sleep(0.1)
            continue
        success, corners = cv2.findChessboardCorners(
            frame,
            chessboard,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        out_queue.put((success, corners))


def main():
    rospy.init_node("record_calibration")

    camera_num = rospy.get_param("~camera_num", 0)
    directory = rospy.get_param("~directory", "")
    name_prefix = rospy.get_param("~prefix", "camera")
    board_width = rospy.get_param("~board_width", 5)
    board_height = rospy.get_param("~board_height", 8)

    write_directory = os.path.join(directory, f"{name_prefix}_{camera_num}")
    chessboard = (board_width, board_height)
    window_name = "calibration"

    if not os.path.isdir(write_directory):
        os.makedirs(write_directory)
        print("Making directory:", write_directory)

    data = AppData(window_name, CvBridge(), threading.Lock(), chessboard)
    in_queue = Queue()
    out_queue = Queue()
    detection_thread = threading.Thread(
        target=detection_task, args=(data, in_queue, out_queue)
    )
    detection_thread.daemon = True
    detection_thread.start()

    cv2.namedWindow(window_name)

    subscriber = rospy.Subscriber(
        "image_raw", Image, lambda msg: image_callback(data, msg), queue_size=1
    )
    image_count = 0

    found_corners = None

    try:
        while True:
            with data.lock:
                frame = data.frame
                if len(frame) == 0:
                    frame = np.zeros((300, 300), np.uint8)
            if rospy.is_shutdown():
                break
            in_queue.put(frame)
            if not out_queue.empty():
                success, corners = out_queue.get()
                if success:
                    found_corners = corners

            debug = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            if found_corners is not None:
                debug = cv2.drawChessboardCorners(
                    debug, chessboard, found_corners, True
                )

            cv2.imshow(window_name, debug)
            key = chr(cv2.waitKey(1) & 0xFF)
            if key == "q":
                print("Exiting")
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
