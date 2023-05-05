from typing import Optional
import numpy as np
import yaml
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest

from utils import ArduCamPixelFormat, ArducamUtils


class ArduCamQuad:
    def __init__(self) -> None:
        rospy.init_node("ardu_cam_quad")
        self.device = int(rospy.get_param("~device", 0))
        self.camera_info_path = str(rospy.get_param("~camera_info_path", 0))
        self.channel = int(rospy.get_param("~channel", -1))
        self.pixel_format = ArduCamPixelFormat(rospy.get_param("~pixel_format", "raw8"))

        self.bridge = CvBridge()
        self.open()
        self.info = self.load_camera_info(self.camera_info_path)

    def open(self) -> None:
        self.arducam = ArducamUtils(self.device, self.pixel_format)

    def close(self) -> None:
        self.arducam.close()

    def load_camera_info(self, path: str) -> CameraInfo:
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
            frame = self.arducam.get()
            if frame is None:
                rospy.logerr("Failed to get arducam frame. Reopening interface.")
                self.close()
                self.open()
                continue
            message = self.numpy_to_ros_image(frame)
            if message is None:
                continue

    def numpy_to_ros_image(self, image: np.ndarray) -> Optional[Image]:
        try:
            return self.bridge.cv2_to_imgmsg(image, self.arducam.get_ros_encoding())
        except CvBridgeError as e:
            rospy.logerr("Failed to convert arducam array to ROS image", exc_info=e)
            return None
