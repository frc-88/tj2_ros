from dataclasses import dataclass

from geometry_msgs.msg import Quaternion, Vector3

from tj2_tools.transforms.transform3d import Transform3D


@dataclass
class TagConfig:
    id: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 0.0
    size: float = 0.0

    def __post_init__(self) -> None:
        self.transform = Transform3D.from_position_and_quaternion(
            Vector3(x=self.x, y=self.y, z=self.z),
            Quaternion(x=self.qx, y=self.qy, z=self.qz, w=self.qw),
        )
