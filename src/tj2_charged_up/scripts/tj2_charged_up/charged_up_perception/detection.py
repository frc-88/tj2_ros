import numpy as np
from dataclasses import dataclass
from geometry_msgs.msg import Quaternion


@dataclass
class BoundingBox2d:
    x_right: float
    y_top: float
    x_left: float
    y_bottom: float


@dataclass
class BoundingBox3d:
    x: float
    y: float
    z: float
    width: float
    height: float
    depth: float


@dataclass
class Detection2d:
    label: str
    index: int
    bounding_box: BoundingBox2d
    angle_degrees: float
    is_standing: bool
    is_obstructed: bool
    mask: np.ndarray


@dataclass
class Detection3d:
    label: str
    index: int
    orientation: Quaternion
    bounding_box: BoundingBox3d
