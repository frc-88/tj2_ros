#!/usr/bin/python
import time

import cv2
import numpy as np
import rospy
import tqdm
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class PublishVideo:
    def __init__(self) -> None:
        path = rospy.get_param("~path")
        self.frames = []
        self.video = cv2.VideoCapture(path)
        self.bridge = CvBridge()
        self.length = self.video.get(cv2.CAP_PROP_FRAME_COUNT)
        self.tick_delay = 1 / self.video.get(cv2.CAP_PROP_FPS)
        self.pbar = tqdm.tqdm(
            desc="image", total=self.length, bar_format="Video: {desc}{percentage:3.0f}%|{bar}|{n}/{total}"
        )
        self.image_pub = rospy.Publisher("image", Image, queue_size=1)

    def publish_image(self, image: np.ndarray):
        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            t0 = time.perf_counter()
            success, frame = self.video.read()
            if not success:
                rospy.loginfo("Video ended")
                break
            self.publish_image(frame)
            t1 = time.perf_counter()

            self.pbar.update(1)
            diff_time = t1 - t0
            remaining_time = self.tick_delay - diff_time
            if remaining_time > 0:
                time.sleep(remaining_time)


if __name__ == "__main__":
    rospy.init_node("publish_video")
    PublishVideo().run()
