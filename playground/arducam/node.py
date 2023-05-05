#!/usr/bin/env python3
import os
import copy
from typing import List, Optional

import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from utils import Arducam, ArduCamPixelFormat


def copy_image_metadata(message: Image) -> Image:
    image = Image()
    image.height = message.height
    image.width = message.width
    image.header = copy.copy(message.header)
    image.encoding = message.encoding
    image.is_bigendian = message.is_bigendian
    return image


class ArduCamQuad:
    def __init__(self) -> None:
        rospy.init_node("ardu_cam_quad")
        self.device = int(rospy.get_param("~device", 0))
        self.camera_info_directory = str(rospy.get_param("~camera_info_directory", "."))
        self.channel = int(rospy.get_param("~channel", -1))
        self.width = int(rospy.get_param("~width", -1))
        self.height = int(rospy.get_param("~height", -1))
        self.publish_combined = bool(rospy.get_param("~publish_combined", False))
        self.pixel_format = ArduCamPixelFormat(rospy.get_param("~pixel_format", "raw8"))

        self.bridge = CvBridge()
        self.open()
        self.infos: List[Optional[CameraInfo]] = []
        self.image_publishers: List[rospy.Publisher] = []
        self.info_publishers: List[rospy.Publisher] = []

        self.combined_publisher = rospy.Publisher(
            "combined_camera/image_raw", Image, queue_size=1
        )
        for index in range(Arducam.NUM_CAMERAS):
            path = os.path.join(self.camera_info_directory, f"camera-{index}.yaml")
            info = self.load_camera_info(path)
            self.infos.append(info)

            image_publisher = rospy.Publisher(
                f"camera_{index}/image_raw", Image, queue_size=1
            )
            info_publisher = rospy.Publisher(
                f"camera_{index}/camera_info", CameraInfo, queue_size=1
            )
            self.image_publishers.append(image_publisher)
            self.info_publishers.append(info_publisher)

    def open(self) -> None:
        self.arducam = Arducam(self.device, self.pixel_format)

    def close(self) -> None:
        self.arducam.close()

    def load_camera_info(self, path: str) -> Optional[CameraInfo]:
        if not os.path.isfile(path):
            rospy.logwarn(
                f"No camera info found. {path} doesn't exist. Not publishing camera info."
            )
            return None
        with open(path) as file:
            config = yaml.safe_load(file)
        info = CameraInfo()
        info.height = config.get("height")
        info.width = config.get("width")
        info.distortion_model = config.get("distortion_model", "plumb_bob")
        info.D = config.get("D")
        info.K = config.get("K")
        info.R = config.get("R")
        info.P = config.get("P")
        info.binning_x = config.get("binning_x", 0)
        info.binning_y = config.get("binning_y", 0)
        roi = config.get("roi", None)
        if roi is not None:
            info.roi = RegionOfInterest(
                x_offset=roi.get("x_offset", 0),
                y_offset=roi.get("y_offset", 0),
                height=roi.get("height", 0),
                width=roi.get("width", 0),
                do_rectify=roi.get("do_rectify", False),
            )
        return info

    def run(self) -> None:
        while not rospy.is_shutdown():
            combined_frame = self.arducam.get_combined()
            if combined_frame is None:
                rospy.logerr("Failed to get arducam frame. Exiting.")
                self.close()
                break

            # combined_message = self.numpy_to_ros_image(combined_frame)
            # if self.publish_combined:
            #     self.combined_publisher.publish(combined_message)
            frames = self.arducam.split_frame(combined_frame)

            for index in range(self.arducam.NUM_CAMERAS):
                # message = copy_image_metadata(combined_message)
                # message.data = split_data[index]
                # message.step = len(message.data) // message.height
                message = self.numpy_to_ros_image(frames[index])
                self.image_publishers[index].publish(message)

                info = self.infos[index]
                if info is not None:
                    self.info_publishers[index].publish(info)

    def numpy_to_ros_image(self, image: np.ndarray) -> Image:
        try:
            return self.bridge.cv2_to_imgmsg(image, "mono8")
        except CvBridgeError as e:
            rospy.logerr("Failed to convert arducam array to ROS image", exc_info=e)
            raise


if __name__ == "__main__":
    node = ArduCamQuad()
    node.run()
