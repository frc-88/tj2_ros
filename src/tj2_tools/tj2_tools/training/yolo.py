import copy
from typing import Dict, List, Tuple
from .training_object import TrainingFrame, TrainingObject
from .get_image_size import get_image_size

class YoloObject(TrainingObject):
    def __init__(self, label, x0, x1, y0, y1, width, height):
        super().__init__(label, x0, x1, y0, y1, width, height)

    @staticmethod
    def _format_bndbox_element(x):
        return "%0.6f" % x

    def _to_yolo_box(self) -> Tuple[float, float, float, float]:
        bounding_box = self.bounding_box.constrain()
        center_x = (bounding_box.x1 + bounding_box.x0) / 2.0
        center_y = (bounding_box.y1 + bounding_box.y0) / 2.0
        box_width = bounding_box.x1 - bounding_box.x0
        box_height = bounding_box.y1 - bounding_box.y0
        center_x /= bounding_box.width
        center_y /= bounding_box.height
        box_width /= bounding_box.width
        box_height /= bounding_box.height
        return center_x, center_y, box_width, box_height

    def to_yolo(self, label_index_mapping: Dict[str, int]) -> str:
        class_index = label_index_mapping[self.label]
        string = str(class_index) + " "
        string += " ".join(map(self._format_bndbox_element, self._to_yolo_box()))
        return string

    @classmethod
    def from_str(cls, line, labels: List[str], width, height):
        elements = line.split(" ")
        index = int(elements[0].strip())
        label = labels[index]
        
        box = [float(element.strip()) for element in elements[1:]]
        assert len(box) == 4
        center_x = box[0]
        center_y = box[1]
        box_width = box[2]
        box_height = box[3]
        x0 = (center_x - box_width / 2.0) * width
        x1 = (center_x + box_width / 2.0) * width
        y0 = (center_y - box_height / 2.0) * height
        y1 = (center_y + box_height / 2.0) * height
        
        return cls(label, x0, x1, y0, y1, width, height)


class YoloFrame(TrainingFrame):
    def __init__(self, frame_path: str, image_path: str, labels: List[str]):
        super().__init__(frame_path, image_path)
        self.labels = labels
        self.label_index_mapping = {label: index for index, label in enumerate(labels)}

    @classmethod
    def from_frame(cls, other: "YoloFrame"):
        assert type(other) == YoloFrame
        self = cls(other.frame_path, other.image_path, other.labels)
        for obj in other.objects:
            self.add_object(copy.deepcopy(obj))
        return self

    def read(self):
        width, height = get_image_size(self.image_path)
        with open(self.frame_path) as file:
            for line in file.read().splitlines():
                obj = YoloObject.from_str(line, self.labels, width, height)
                self.add_object(obj)

    @classmethod
    def from_file(cls, frame_path: str, image_path: str, labels: List[str]):
        self = cls(frame_path, image_path, labels)
        self.read()
        return self

    def write(self):
        contents = ""
        for obj in self.objects:
            assert type(obj) == YoloObject
            contents += obj.to_yolo(self.label_index_mapping) + "\n"
        with open(self.frame_path, 'w') as file:
            file.write(contents)
