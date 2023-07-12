from dataclasses import dataclass
import uvc
from uvc.uvc_bindings import CameraMode


@dataclass
class CameraConfig:
    width: int
    height: int
    fps: int



class UVCCamera:
    def __init__(self) -> None:
        pass
