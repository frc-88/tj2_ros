#!/usr/bin/env python3
import rospy
from typing import Optional, Dict

from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager  # type: ignore

from tj2_northstar.uvc_camera import UVCCamera, CaptureConfig, CameraConfig


class CameraPublisher:
    def __init__(self, name: str) -> None:
        assert "/" not in name, "Camera name cannot contain a slash"
        self.name = name
        self.info_manager: Optional[CameraInfoManager] = None
        image_topic = self.name + "/image_raw"
        info_topic = self.name + "/camera_info"
        self.image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
        self.info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=1)

    def load_info(self, url: str) -> None:
        namespace = rospy.get_namespace() + "/" + self.name + "/"
        while namespace.startswith("/"):
            namespace = namespace[1:]
        namespace = "/" + namespace
        self.info_manager = CameraInfoManager(self.name, url=url, namespace=namespace)
        self.info_manager.loadCameraInfo()

    def publish(self, image: Image) -> None:
        if self.info_manager is not None:
            info = self.info_manager.getCameraInfo()
            info.header = image.header
            self.info_pub.publish(info)
        self.image_pub.publish(image)


class UVCCameraNode:
    """
    This ROS node connects to one or more UVC cameras, applies the requested modees and controls,
    and publishes the images to ROS topics. In addition, it will take the supplied camera info if provided and
    publish it to the camera_info topic.
    """

    def __init__(self) -> None:
        rospy.init_node("uvc_camera_node")

        camera_configs = rospy.get_param("~cameras", default={})
        info_urls = rospy.get_param("~info_urls", default={})
        self.frame_timeout = rospy.get_param("~frame_timeout", default=1.0)
        self.tick_rate = rospy.get_param("~tick_rate", default=0.0)

        if len(camera_configs) == 0:
            raise ValueError("No cameras specified in the camera config")

        self.cameras: Dict[str, UVCCamera] = {}
        self.publishers: Dict[str, CameraPublisher] = {}
        for name, config in camera_configs.items():
            camera = UVCCamera(
                config["frame_id"], CaptureConfig.from_dict(config["capture"])
            )
            camera.set_mode(CameraConfig.from_dict(config["mode"]))
            camera.set_controls(config["controls"])
            self.cameras[name] = camera
            self.publishers[name] = CameraPublisher(name)

        for name, url in info_urls.items():
            self.publishers[name].load_info(url)

    def run(self) -> None:
        if self.tick_rate > 0.0:
            self.rate = rospy.Rate(self.tick_rate)
        else:
            self.rate = None
        while not rospy.is_shutdown():
            for name, camera in self.cameras.items():
                frame = camera.get_frame(self.frame_timeout)
                if frame is not None:
                    self.publishers[name].publish(frame)
            if self.rate is not None:
                self.rate.sleep()
        for camera in self.cameras.values():
            camera.close()


if __name__ == "__main__":
    node = UVCCameraNode()
    node.run()
