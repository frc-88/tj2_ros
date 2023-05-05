import os
from typing import Optional, List
import numpy as np
import yaml
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest

from utils import ArduCamPixelFormat, Arducam


class ArduCamQuad:
    def __init__(self) -> None:
        rospy.init_node("ardu_cam_quad")
        self.device = int(rospy.get_param("~device", 0))
        self.camera_info_directory = str(rospy.get_param("~camera_info_directory", "."))
        self.channel = int(rospy.get_param("~channel", -1))
        self.width = int(rospy.get_param("~width", -1))
        self.height = int(rospy.get_param("~height", -1))
        self.pixel_format = ArduCamPixelFormat(rospy.get_param("~pixel_format", "raw8"))

        self.bridge = CvBridge()
        self.open()
        self.infos: List[CameraInfo] = []
        self.image_publishers: List[rospy.Publisher] = []
        self.info_publishers: List[rospy.Publisher] = []

        for index in range(Arducam.NUM_CAMERAS):
            path = os.path.join(self.camera_info_directory, f"camera-{index}.yaml")
            info = self.load_camera_info(path)
            self.infos.append(info)

            image_publisher = rospy.Publisher(
                f"camera_{index}/image_rect", Image, queue_size=1
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
        if roi := config.get("roi", None):
            info.roi = RegionOfInterest(
                x_offset=roi.get("x_offset", 0),
                y_offset=roi.get("y_offset", 0),
                height=roi.get("height", 0),
                width=roi.get("width", 0),
                do_rectify=roi.get("do_rectify", False),
            )
        return info

    def run(self) -> None:
        while True:
            frames = self.arducam.get()
            if frames is None:
                rospy.logerr("Failed to get arducam frame. Reopening interface.")
                self.close()
                rospy.sleep(0.25)
                self.open()
                rospy.sleep(0.25)
                continue
            for index, frame in enumerate(frames):
                info = self.infos[index]
                message = self.numpy_to_ros_image(frame)
                if message is None:
                    continue
                self.image_publishers[index].publish(message)
                if info is not None:
                    self.info_publishers[index].publish(info)

    def get_ros_encoding(self) -> str:
        if self.arducam.convert2rgb == 0:
            channels = "mono"
        else:
            channels = "rgb"
        if self.depth == 16:
            depth = "16"
        else:
            depth = "8"
        return channels + depth

    def numpy_to_ros_image(self, image: np.ndarray) -> Optional[Image]:
        try:
            return self.bridge.cv2_to_imgmsg(image, self.get_ros_encoding())
        except CvBridgeError as e:
            rospy.logerr("Failed to convert arducam array to ROS image", exc_info=e)
            return None
