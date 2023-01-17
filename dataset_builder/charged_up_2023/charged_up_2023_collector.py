import os
import json
from typing import Generator, List, Tuple
from tj2_tools.training.dataset_builder.collector import Collector
from tj2_tools.training.training_object import TrainingFrame
from tj2_tools.training.yolo import YoloFrame, YoloObject
from tj2_tools.training.get_image_size import get_image_size


def read_labelme_annotation(labelme_path, image_path, labels) -> YoloFrame:
    anno_path = os.path.splitext(image_path)[0] + ".txt"
    anno = YoloFrame(anno_path, image_path, labels)
    
    with open(labelme_path) as file:
        data = json.load(file)
    
    width, height = get_image_size(image_path)
    for shape in data["shapes"]:
        if shape["shape_type"] != "rectangle":
            continue
        x0 = shape["points"][0][0]
        y0 = shape["points"][0][1]
        x1 = shape["points"][1][0]
        y1 = shape["points"][1][1]
        
        label = shape["label"]
        
        anno.add_object(YoloObject(label, x0, x1, y0, y1, width, height))
    
    return anno


class ChargedUp2023Collector(Collector):
    def __init__(self, base_path, labels: List[str]) -> None:
        self.base_path = base_path
        self.labels = labels

        self.frame_paths = []
        for dirpath, dirnames, filenames in os.walk(self.base_path):
            for filename in filenames:
                path = os.path.join(dirpath, filename)
                if filename.endswith(".txt") or filename.endswith(".json"):
                    self.frame_paths.append(path)
        self.frame_paths.sort()
    
    def iter(self) -> Generator[TrainingFrame, None, None]:
        for path in self.frame_paths:
            image_paths = self.find_image(path)
            assert len(image_paths) != 0
            if path.endswith(".txt"):
                frame = YoloFrame.from_file(path, image_paths[0], self.labels)
            elif path.endswith(".json"):
                frame = read_labelme_annotation(path, image_paths[0], self.labels)
            else:
                continue
            if frame.get_num_objects() == 0:
                print(f"Skipping {path}. No objects in frame.")
                continue
            yield frame

    def get_length(self) -> int:
        return len(self.frame_paths)

    def find_image(self, frame_path: str) -> Tuple[str, ...]:
        frame_name = os.path.splitext(frame_path)[0]
        dirname = os.path.dirname(frame_path)
        image_paths = []
        for filename in os.listdir(dirname):
            name, ext = os.path.splitext(filename)
            if frame_name.endswith(name):
                if ext.lower() in (".jpg", ".png"):
                    image_paths.append(os.path.join(dirname, filename))
        return tuple(image_paths)
