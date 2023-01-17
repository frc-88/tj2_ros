import os
import copy
from typing import List


class BoundingBox:
    def __init__(self, x0: float, x1: float, y0: float, y1: float, width: float, height: float) -> None:
        self.x0 = x0
        self.x1 = x1
        self.y0 = y0
        self.y1 = y1
        self.width = width
        self.height = height

    def constrain(self) -> "BoundingBox":
        lower_x = min(self.x0, self.x1)
        upper_x = max(self.x0, self.x1)
        lower_y = min(self.y0, self.y1)
        upper_y = max(self.y0, self.y1)
        
        lower_x = max(0.0, min(self.width, lower_x))
        upper_x = max(0.0, min(self.width, upper_x))
        lower_y = max(0.0, min(self.height, lower_y))
        upper_y = max(0.0, min(self.height, upper_y))
        
        return BoundingBox(lower_x, upper_x, lower_y, upper_y, self.width, self.height)

    def __repr__(self) -> str:
        return (
            f"{self.__class__.__name__}("
            f"x0={self.x0}, x1={self.x1}), "
            f"y0={self.y0}), y1={self.y1}), "
            f"width={self.width}), "
            f"height={self.height})"
        )
    
    def __str__(self) -> str:
        return f"({int(self.x0)}, {int(self.y0)})..({int(self.x1)}, {int(self.y1)})|({int(self.width)}x{int(self.height)})"


class TrainingObject:
    def __init__(self, label: str, x0: float, x1: float, y0: float, y1: float, width: float, height: float) -> None:
        self.bounding_box = BoundingBox(x0, x1, y0, y1, width, height)
        self.label = label

    def get_label(self):
        return self.label
    
    def __str__(self) -> str:
        return f"<{self.label} {str(self.bounding_box)}>"


class TrainingFrame:
    def __init__(self, frame_path: str, image_path: str) -> None:
        self.frame_path = frame_path
        self.image_path = image_path
        self.objects: List[TrainingObject] = []

    @classmethod
    def from_frame(cls, other: "TrainingFrame"):
        self = cls(other.frame_path, other.image_path)
        for obj in other.objects:
            self.add_object(copy.deepcopy(obj))

    def __hash__(self) -> int:
        return hash(self.frame_path)

    def __eq__(self, other):
        return isinstance(other, TrainingFrame) and self.frame_path == other.frame_path

    def get_object(self, index: int) -> TrainingObject:
        return self.objects[index]

    def get_num_objects(self) -> int:
        return len(self.objects)

    def add_object(self, obj: TrainingObject) -> int:
        self.objects.append(obj)
        return len(self.objects)

    def set_paths(self, frame_path, image_path):
        self.image_path = image_path
        self.frame_path = frame_path

    def write(self):
        raise NotImplementedError

    def __str__(self) -> str:
        objects_str = " | ".join([str(obj) for obj in self.objects])
        return f"{os.path.basename(self.frame_path)}: {objects_str}"
