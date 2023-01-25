import os
import tqdm
import jsonlines
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
        self.labels = labels
        self.duplicate_image_name_counts = {}
        self.written_duplicate_counts = {}
        self.written_annotation_paths = set()
        self.written_images_paths = set()

    def reset(self):
        # deletes all image and xml files under annotations and jpeg images respectively
        annotations_dir = self.output_dir / ANNOTATIONS
        images_dir = self.output_dir / JPEGIMAGES
        self.reset_dir(annotations_dir, ".txt")
        self.reset_dir(images_dir, ".jpg", ".jpeg")
        self.duplicate_image_name_counts = {}
        self.written_annotation_paths = set()
        self.written_images_paths = set()

    def build(self):
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

        # erase metadata files
        for key in image_sets.keys():
            with open(self.get_metadata_path(key), 'w') as file:
                file.write('')

        count = 0
        for key, image_set in image_sets.items():
            for frame_hash in image_set:
                frame = frame_map[frame_hash]
                self.write_metadata(self.get_metadata_path(key), frame)
                self.copy_annotation(key, frame)
                count += 1
        print(f"Copied {count} annotations")

        self.write_labels()

    def get_metadata_path(self, key):
        return self.output_dir / JPEGIMAGES / key / "metadata.jsonl"

    def write_metadata(self, path, frame: YoloFrame):
        bbox = []
        categories = []
        for obj in frame.objects:
            index = self.labels.index(obj.label)
            bbox.append([
                obj.bounding_box.x0,
                obj.bounding_box.y0,
                obj.bounding_box.x1,
                obj.bounding_box.y1,
            ])
            categories.append(index)
        data = {
            "file_name": os.path.basename(frame.image_path),
            "objects": {
                "bbox": bbox,
                "categories": categories
            }
        }
        if not self.dry_run:
            with jsonlines.open(path, mode='a') as writer:
                writer.write(data)

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
        
        if filename_count > 0:
            if image_filename not in self.written_duplicate_counts:
                self.written_duplicate_counts[image_filename] = 0
            counter = self.written_duplicate_counts[image_filename]
            self.written_duplicate_counts[image_filename] += 1
        else:
            counter = 0
        
        new_image_path = images_dir / image_filename
        new_anno_path = annotation_dir / os.path.basename(frame.frame_path)
        if counter > 0:
            name = new_image_path.stem
            ext = new_image_path.suffix
            new_image_path = new_image_path.parent / ("%s-%05d%s" % (name, counter, ext))

            name = new_anno_path.stem
            ext = ".txt"
            new_anno_path = new_anno_path.parent / ("%s-%05d%s" % (name, counter, ext))
        else:
            name = new_anno_path.stem
            ext = ".txt"
            new_anno_path = new_anno_path.parent / ("%s%s" % (name, ext))

        if not self.dry_run:
            shutil.copy(frame.image_path, new_image_path)

        new_anno_path_str = str(new_anno_path.absolute())
        new_image_path_str = str(new_image_path.absolute())
        if new_anno_path_str in self.written_annotation_paths:
            print(f"WARNING: overwriting {new_anno_path_str}")
        if new_image_path_str in self.written_images_paths:
            print(f"WARNING: overwriting {new_image_path_str}")
        self.written_annotation_paths.add(new_anno_path_str)
        self.written_images_paths.add(new_image_path_str)

        copy_frame = YoloFrame.from_frame(frame)
        copy_frame.set_paths(new_anno_path_str, new_image_path_str)
        if not self.dry_run:
            copy_frame.write()
