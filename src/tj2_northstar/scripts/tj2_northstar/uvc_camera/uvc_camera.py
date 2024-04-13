import copy
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Union

import rospy
import uvc
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from uvc.uvc_bindings import CameraMode

from tj2_northstar.uvc_camera.helper import find_match


class CameraFormat(Enum):
    MJPG = "MJPG"
    YUYV = "YUYV"


@dataclass
class CameraConfig:
    width: int
    height: int
    fps: int
    format: CameraFormat

    @classmethod
    def from_dict(cls, d: Dict[str, Union[str, int]]) -> "CameraConfig":
        d = copy.copy(d)
        self = cls(
            width=int(d.pop("width")),
            height=int(d.pop("height")),
            fps=int(d.pop("fps")),
            format=CameraFormat(d.pop("format")),
        )
        if len(d) != 0:
            raise ValueError(f"Dictionary contained extra values for CameraConfig: {d}")
        return self


@dataclass
class CaptureConfig:
    serial_number: str = ""
    uid: str = ""
    product_id: str = ""
    vendor_id: str = ""
    index: int = 0

    @classmethod
    def from_dict(cls, d: Dict[str, Union[str, int]]) -> "CaptureConfig":
        d = copy.copy(d)
        self = cls(
            serial_number=str(d.pop("serial_number", "")),
            uid=str(d.pop("uid", "")),
            product_id=str(d.pop("product_id", "")),
            vendor_id=str(d.pop("vendor_id", "")),
            index=int(d.pop("index", 0)),
        )
        if len(d) != 0:
            raise ValueError(f"Dictionary contained extra values for CameraConfig: {d}")
        return self


@dataclass(eq=True, frozen=True)
class CaptureInfo:
    name: str
    manufacturer: str
    serialNumber: str
    idProduct: int
    idVendor: int
    device_address: int
    bus_number: int
    uid: str

    @classmethod
    def from_dict(cls, d: Dict[str, Union[str, int]]) -> "CaptureInfo":
        return cls(
            name=str(d["name"]),
            manufacturer=str(d["manufacturer"]),
            serialNumber=str(d["serialNumber"]),
            idProduct=int(d["idProduct"]),
            idVendor=int(d["idVendor"]),
            device_address=int(d["device_address"]),
            bus_number=int(d["bus_number"]),
            uid=str(d["uid"]),
        )

    def __lt__(self, other: Any) -> bool:
        if not isinstance(other, CaptureInfo):
            return False
        else:
            return self.get_id() < other.get_id()

    def get_id(self) -> int:
        return (self.idProduct << 16) | self.idVendor


Controls = Dict[str, int]


class UVCCamera:
    bridge = CvBridge()
    opened_uids = set()

    def __init__(self, frame_id: str, serial_number: str) -> None:
        self.serial_number = serial_number
        self.frame_id = frame_id
        self.error_status = ""
        info = self.get_capture_info(self.serial_number)
        if info.uid in UVCCamera.opened_uids:
            raise ConnectionError(f"Camera is already opened: {info}. Supplied serial: {self.serial_number}")
        UVCCamera.opened_uids.add(info.uid)
        rospy.loginfo(f"Opening {info}")
        self.capture = uvc.Capture(info.uid)

    def get_modes(self) -> List[CameraMode]:
        return self.capture.available_modes

    def get_mode(self) -> CameraMode:
        return self.capture.frame_mode

    def set_mode(self, camera_config: CameraConfig) -> None:
        self.capture.frame_mode = self.select_mode(self.capture, camera_config)

    def get_controls(self) -> Controls:
        return self.capture.controls

    def set_controls(self, controls: Controls) -> None:
        if len(controls) == 0:
            return
        for control in self.capture.controls:
            if control.display_name in controls:
                control.value = controls[control.display_name]

    @classmethod
    def get_available_captures(cls) -> List[CaptureInfo]:
        return [CaptureInfo.from_dict(x) for x in uvc.device_list()]

    def is_capture_check(self, capture_info: CaptureInfo, capture_config: CaptureConfig) -> int:
        count = 0
        if capture_info.uid == capture_config.uid:
            count += 1
        if capture_info.serialNumber == capture_config.serial_number:
            count += 1
        if capture_info.idVendor == capture_config.vendor_id and capture_info.idProduct == capture_config.product_id:
            count += 1
        return count

    def get_capture_info(self, serial_number: str) -> CaptureInfo:
        available_captures = self.get_available_captures()
        selected_capture = None
        for capture in available_captures:
            if capture.serialNumber == serial_number:
                selected_capture = capture
        if selected_capture is None:
            rospy.loginfo("Available captures: %s" % available_captures)
            raise RuntimeError("Failed to find capture with serial %s" % serial_number)
        return selected_capture

    def is_mode_match(self, mode: CameraMode, config: CameraConfig) -> int:
        return (
            int(config.width == mode.width)
            + int(config.height == mode.height)
            + int(config.fps == mode.fps)
            + int(config.format.value == mode.format_name)
        )

    def select_mode(self, capture: uvc.Capture, camera_config: CameraConfig) -> CameraMode:
        for mode in capture.available_modes:
            rospy.loginfo("Available mode: %s" % str(mode))
        modes = find_match(capture.available_modes, camera_config, self.is_mode_match)
        return modes[0]

    def get_error(self) -> str:
        return self.error_status

    def get_frame(self, timeout: float = 1.0) -> Optional[Image]:
        try:
            frame = self.capture.get_frame(timeout=timeout)
        except TimeoutError as e:
            self.error_status = "%s: Camera %s timed out." % (str(e), self.serial_number)
            rospy.logerr(self.error_status, exc_info=e)
            return None
        self.error_status = ""
        is_bgr = hasattr(frame, "bgr")
        data = frame.bgr if is_bgr else frame.gray
        encoding = "bgr8" if is_bgr else "mono8"
        if frame.data_fully_received:
            return self.bridge.cv2_to_imgmsg(
                data,
                encoding=encoding,
                header=Header(frame_id=self.frame_id, stamp=rospy.Time.now()),
            )
        else:
            self.error_status = "Image not fully received."
            return None

    def close(self) -> None:
        self.capture.close()
