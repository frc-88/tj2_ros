import os
from typing import Generator, List, Tuple
from tj2_tools.training.yolo import YoloFrame
from tj2_tools.training.dataset_builder.collector import Collector
from tj2_tools.training.training_object import TrainingFrame
from util import read_labelme_annotation


class ChargedUp2023Collector(Collector):
    def __init__(self, base_path, labels: List[str], images_path: str = "") -> None:
        self.base_path = base_path
        self.images_path = images_path
        self.labels = labels

        self.frame_paths = []
        for dirpath, dirnames, filenames in os.walk(self.base_path):
            for filename in filenames:
                path = os.path.join(dirpath, filename)
                if filename.endswith(".txt") or filename.endswith(".json"):
                    self.frame_paths.append(path)
        self.frame_paths.sort()
        
        if len(images_path) == 0:
            self.images_cache = {}
        else:
            self.images_cache = self.generate_image_path_cache(self.images_path)
    
    def generate_image_path_cache(self, images_path):
        cache = {}
        for dirpath, dirnames, filenames in os.walk(images_path):
            for filename in filenames:
                if filename.lower().endswith(".jpg") or filename.lower().endswith(".png"):
                    path = os.path.join(dirpath, filename)
                    name = os.path.splitext(filename)[0]
                    cache[name] = path
        return cache
    
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
        name = os.path.splitext(os.path.basename(frame_path))[0]
        if name in self.images_cache:
            return (self.images_cache[name],)
        else:
            frame_name = os.path.splitext(frame_path)[0]
            dirname = os.path.dirname(frame_path)
            image_paths = []
            for filename in os.listdir(dirname):
                name, ext = os.path.splitext(filename)
                if frame_name.endswith(name):
                    if ext.lower() in (".jpg", ".png"):
                        image_paths.append(os.path.join(dirname, filename))
            return tuple(image_paths)
