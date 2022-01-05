#!/usr/bin/python3
import rospy

import os
import cv2
import time
import yaml
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class LimelightImagePublisher:
    def __init__(self):
        self.node_name = "limelight_image_publisher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.bridge = CvBridge()

        self.images_dir = rospy.get_param("~images_dir", "./images")
        self.camera_info_path = rospy.get_param("~camera_info_path", "./config/320x240.yaml")

        self.image_paths = self.load_image_paths(self.images_dir)
        self.frame_dwell = 2.0
        self.fps = 10
        self.image_index = 0
        self.dwell_timer = time.time()

        self.camera_info = self.load_camera_info(self.camera_info_path)

        self.color_image_pub = rospy.Publisher("color/image_raw", Image, queue_size=10)
        self.color_info_pub = rospy.Publisher("color/camera_info", CameraInfo, queue_size=10)
        self.depth_image_pub = rospy.Publisher("aligned_depth_to_color/image_raw", Image, queue_size=10)
        self.depth_info_pub = rospy.Publisher("aligned_depth_to_color/camera_info", CameraInfo, queue_size=10)

        rospy.loginfo("%s init complete" % self.node_name)
    
    def load_image_paths(self, images_dir):
        image_paths = []
        for filename in os.listdir(images_dir):
            ext = os.path.splitext(filename)[-1].lower()
            if not (ext == ".jpg" or ext == ".jpeg"):
                continue
            path = os.path.join(images_dir, filename)
            image_paths.append(path)
        return image_paths

    def run(self):
        while not rospy.is_shutdown():
            frame = self.get_next_color_frame()
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            except TypeError as e:
                rospy.logerr("Exception occurred while converting frame: %s. %s" % (e, frame))
                continue
            msg.header = self.camera_info.header
            self.color_image_pub.publish(msg)

            frame = self.get_next_depth_frame()
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            except TypeError as e:
                rospy.logerr("Exception occurred while converting frame: %s. %s" % (e, frame))
                continue
            msg.header = self.camera_info.header
            self.depth_image_pub.publish(msg)

            self.color_info_pub.publish(self.camera_info)
            self.depth_info_pub.publish(self.camera_info)

    def get_next_color_frame(self):
        time.sleep(1.0 / self.fps)
        path = self.image_paths[self.image_index]
        if self.frame_dwell > 0.0:
            if time.time() - self.dwell_timer > self.frame_dwell:
                self.dwell_timer = time.time()
                self.image_index += 1
        else:
            self.image_index += 1
        if self.image_index >= len(self.image_paths):
            self.image_index = 0
        assert os.path.isfile(path), path
        frame = cv2.imread(path)
        if frame is None:
            rospy.logerr("%s did not load" % str(path))
        return frame
    
    def get_next_depth_frame(self):
        depth_frame = np.zeros((self.camera_info.height, self.camera_info.width), np.uint16)
        depth_frame[:] = 250
        return depth_frame
    
    def load_camera_info(self, path):
        with open(path) as file:
            config = yaml.safe_load(file.read())
        info = CameraInfo()
        info.height = config["image_height"]
        info.width = config["image_width"]
        info.distortion_model = config["distortion_model"]
        info.D = config["distortion_coefficients"]["data"]
        info.K = config["camera_matrix"]["data"]
        info.R = config["rectification_matrix"]["data"]
        info.P = config["projection_matrix"]["data"]
        info.header.frame_id = "limelight_optical_link"
        return info


if __name__ == "__main__":
    node = LimelightImagePublisher()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
