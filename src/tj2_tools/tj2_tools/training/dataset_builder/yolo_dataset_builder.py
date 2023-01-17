import os
import tqdm
from pathlib import Path
import shutil
from .collector import Collector
from .dataset_builder import DatasetBuilder, BACKGROUND_LABEL
from ..training_object import TrainingFrame
from ..yolo import YoloFrame

ANNOTATIONS = "labels"
JPEGIMAGES = "images"


class YoloDatasetBuilder(DatasetBuilder):
    def __init__(self, output_dir: str, image_collector: Collector, labels: list, 
                 test_ratio=0.05, train_ratio=0.80, validation_ratio=0.15,
                 test_name="test", train_name="train", validation_name="val", dry_run=True):
        super(YoloDatasetBuilder, self).__init__(
            Path(output_dir),
            test_ratio, train_ratio, validation_ratio,
            test_name, train_name, validation_name,
            dry_run
        )
        self.image_collector = image_collector
        self.frames = []
        self.labels = labels
        self.duplicate_image_name_counts = {}

    def reset(self):
        # deletes all image and xml files under annotations and jpeg images respectively
        annotations_dir = self.output_dir / ANNOTATIONS
        images_dir = self.output_dir / JPEGIMAGES
        self.reset_dir(annotations_dir, ".txt")
        self.reset_dir(images_dir, ".jpg", ".jpeg")
        self.frames = []
        self.duplicate_image_name_counts = {}

    def build(self):
        self.frames = []
        
        label_to_identifier_map = {}
        frame_map = {}

        with tqdm.tqdm(total=self.image_collector.get_length()) as pbar:
            for frame in self.image_collector.iter():
                pbar.update(1)
                label = frame.get_object(0).label
                if label not in label_to_identifier_map:
                    label_to_identifier_map[label] = []
                label_to_identifier_map[label].append(hash(frame))
                frame_map[hash(frame)] = frame
                
                image_filename = os.path.basename(frame.image_path)
                if image_filename not in self.duplicate_image_name_counts:
                    self.duplicate_image_name_counts[image_filename] = 0
                self.duplicate_image_name_counts[image_filename] += 1
                
        image_sets = self.get_distributed_sets(label_to_identifier_map)

        test_count = len(image_sets[self.test_name])
        train_count = len(image_sets[self.train_name])
        validation_count = len(image_sets[self.validation_name])
        total_count = test_count + train_count + validation_count
        print(
            f"Total {total_count}:\n"
            f"\tTest: {test_count}\t{test_count / total_count:0.2f}\n"
            f"\tTrain: {train_count}\t{train_count / total_count:0.2f}\n"
            f"\tValidation: {validation_count}\t{validation_count / total_count:0.2f}"
        )

        for set_key in image_sets.keys():
            for label in self.labels:
                self.makedir(self.output_dir / ANNOTATIONS / set_key)
                self.makedir(self.output_dir / JPEGIMAGES / set_key)

        for key, image_set in image_sets.items():
            for frame_hash in image_set:
                frame = frame_map[frame_hash]
                self.frames.append(frame)
                self.copy_annotation(key, frame)

        self.write_labels()

    def write_labels(self):
        if BACKGROUND_LABEL in self.labels:
            self.labels.remove(BACKGROUND_LABEL)
        labels_path = self.output_dir / "classes.txt"
        print("Writing labels to %s" % labels_path)
        self.write_list(labels_path, self.labels)

    def copy_annotation(self, subdirectory: str, frame: TrainingFrame):
        annotation_dir = self.output_dir / ANNOTATIONS / subdirectory
        images_dir = self.output_dir / JPEGIMAGES / subdirectory
        self.makedir(annotation_dir)
        self.makedir(images_dir)
        
        image_filename = os.path.basename(frame.image_path)
        filename_count = self.duplicate_image_name_counts[image_filename]
        
        new_image_path = images_dir / image_filename
        if filename_count > 1:
            name = new_image_path.stem
            ext = new_image_path.suffix
            new_image_path = new_image_path.parent / ("%s-%05d%s" % (name, filename_count, ext))

        new_anno_path = annotation_dir / os.path.basename(frame.frame_path)
        if filename_count > 1:
            name = new_anno_path.stem
            ext = ".txt"
            new_anno_path = new_anno_path.parent / ("%s-%05d%s" % (name, filename_count, ext))
        else:
            name = new_anno_path.stem
            ext = ".txt"
            new_anno_path = new_anno_path.parent / ("%s%s" % (name, ext))

        print("Copying image %s -> %s%s" % (frame.image_path, new_image_path,
                                            (". Adding count: %05d" % filename_count) if filename_count > 1 else ""))
        if not self.dry_run:
            shutil.copy(frame.image_path, new_image_path)

        print("Copying annotation %s -> %s" % (frame.frame_path, new_anno_path))
        copy_frame = YoloFrame.from_frame(frame)
        copy_frame.set_paths(str(new_anno_path.absolute()), str(new_image_path.absolute()))
        if not self.dry_run:
            copy_frame.write()
