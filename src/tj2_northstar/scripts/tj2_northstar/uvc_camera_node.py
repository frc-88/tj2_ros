#!/usr/bin/env python3
from typing import Dict, Optional

import rospy
from camera_info_manager import CameraInfoManager  # type: ignore
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import CameraInfo, Image

from tj2_northstar.uvc_camera import CameraConfig, UVCCamera


class CameraPublisher:
    def __init__(self, name: str) -> None:
        assert "/" not in name, "Camera name cannot contain a slash"
        self.name = name
        self.info_manager: Optional[CameraInfoManager] = None
        self.diagnostic_map = {}
        self.diagnostics = DiagnosticStatus()
        image_topic = self.name + "/image_raw"
        info_topic = self.name + "/camera_info"
        diagnostics_topic = self.name + "/diagnostics"
        self.image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
        self.info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=1)
        self.diagnostics_pub = rospy.Publisher(diagnostics_topic, DiagnosticStatus, queue_size=1)

    def load_info(self, url: str) -> None:
        namespace = rospy.get_namespace() + "/" + self.name + "/"
        while namespace.startswith("/"):
            namespace = namespace[1:]
        namespace = "/" + namespace
        self.info_manager = CameraInfoManager(self.name, url=url, namespace=namespace)
        self.info_manager.loadCameraInfo()
        self.diagnostics.name = self.name

    def publish(self, image: Image) -> None:
        self.diagnostics.values = [KeyValue(key, value) for key, value in self.diagnostic_map.items()]
        if self.info_manager is not None:
            info = self.info_manager.getCameraInfo()
            info.header = image.header
            self.info_pub.publish(info)
        self.image_pub.publish(image)
        self.diagnostics_pub.publish(self.diagnostics)

    def set_diagnostic_field(self, key: str, value: str) -> None:
        self.diagnostic_map[key] = value

    def set_diagnostic_id(self, hardware_id: str) -> None:
        self.diagnostics.hardware_id = hardware_id

    def set_diagnostic_level(self, is_ok: bool) -> None:
        self.diagnostics.level = DiagnosticStatus.OK if is_ok else DiagnosticStatus.ERROR


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
        self.frame_timeout = rospy.get_param("~frame_timeout", default=0.1)
        self.tick_rate = rospy.get_param("~tick_rate", default=0.0)

        if len(camera_configs) == 0:
            raise ValueError("No cameras specified in the camera config")

        self.cameras: Dict[str, UVCCamera] = {}
        self.publishers: Dict[str, CameraPublisher] = {}
        for name, config in camera_configs.items():
            rospy.loginfo("Config: %s" % config)
            if len(config["serial_number"]) == 0:
                rospy.logwarn("Serial number supplied is empty. Skipping %s" % name)
                continue
            camera = UVCCamera(config["frame_id"], config["serial_number"])
            camera.set_mode(CameraConfig.from_dict(config["mode"]))
            camera.set_controls(config["controls"])
            self.cameras[name] = camera
            self.publishers[name] = CameraPublisher(name)
            url = info_urls[name]
            self.publishers[name].load_info(url)

    def run(self) -> None:
        rospy.sleep(2.0)  # wait for cameras to come online
        if self.tick_rate > 0.0:
            self.rate = rospy.Rate(self.tick_rate)
        else:
            self.rate = None
        for name, camera in self.cameras.items():
            publisher = self.publishers[name]
            publisher.set_diagnostic_id(camera.serial_number)
            publisher.set_diagnostic_level(True)
            for control in camera.get_controls():
                publisher.set_diagnostic_field(
                    control.display_name,
                    "value: %s, unit: %s, min: %s, max: %s, step: %s"
                    % (control.value, control.unit, control.min_val, control.max_val, control.step),
                )
            mode = camera.get_mode()
            publisher.set_diagnostic_field("width", str(mode.width))
            publisher.set_diagnostic_field("height", str(mode.height))
            publisher.set_diagnostic_field("fps", str(mode.fps))
            publisher.set_diagnostic_field("format_native", str(mode.format_native))
            publisher.set_diagnostic_field("format_name", str(mode.format_name))
            publisher.set_diagnostic_field("supported", str(mode.supported))

        while not rospy.is_shutdown():
            for name, camera in self.cameras.items():
                frame = camera.get_frame(self.frame_timeout)
                if frame is None:
                    publisher.set_diagnostic_level(False)
                    continue
                self.publishers[name].publish(frame)
            if self.rate is not None:
                self.rate.sleep()
        for camera in self.cameras.values():
            camera.close()


if __name__ == "__main__":
    node = UVCCameraNode()
    node.run()
