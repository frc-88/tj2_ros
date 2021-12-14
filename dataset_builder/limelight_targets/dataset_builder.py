import os
import random
from pathlib import Path
import shutil
from tj2_tools.training.pascal_voc import PascalVOCFrame
from image_collector import ImageCollector

ANNOTATIONS = "Annotations"
IMAGESETS = "ImageSets/Main"
JPEGIMAGES = "JPEGImages"
LABELS = "labels.txt"

BACKGROUND_LABEL = "BACKGROUND"


class DatasetBuilder:
    def __init__(self, output_dir: Path, image_collector: ImageCollector, test_ratio=0.15, train_ratio=0.85,
                 validation_ratio=0.0, dry_run=True):
        self.dry_run = dry_run
        self.output_dir = output_dir
        self.makedir(self.output_dir)
        self.image_collector = image_collector
        self.labels = []
        self.frames = []
        self.test_ratio = test_ratio
        self.train_ratio = train_ratio
        self.validation_ratio = validation_ratio

        assert self.check_ratios(self.test_ratio, self.train_ratio, self.validation_ratio), \
            self.test_ratio + self.train_ratio + self.validation_ratio

        self.ratios = {
            "test": self.test_ratio,
            "train": self.train_ratio,
            "validation": self.validation_ratio
        }

    def makedir(self, directory: Path):
        if self.dry_run:
            return
        if not directory.is_dir():
            os.makedirs(directory)

    def build(self):
        self.frames = []
        label_set = set()
        for frame in self.image_collector.iter():
            self.frames.append(frame)
            self.copy_annotation(frame)
            for obj in frame.objects:
                label_set.add(obj.name)

        self.labels = sorted(list(label_set))

        self.build_training_sets()
        self.write_labels()

    def copy_annotation(self, frame: PascalVOCFrame):
        annotation_dir = self.output_dir / ANNOTATIONS
        images_dir = self.output_dir / JPEGIMAGES
        self.makedir(annotation_dir)
        self.makedir(images_dir)
        new_image_path = images_dir / frame.filename
        new_frame_path = annotation_dir / os.path.basename(frame.frame_path)

        print("Copying image %s -> %s" % (frame.path, new_image_path))
        if not self.dry_run:
            shutil.copy(frame.path, new_image_path)

        frame.set_path(new_image_path)
        print("Copying annotation %s -> %s" % (frame.frame_path, new_frame_path))
        frame.frame_path = str(new_frame_path.name)
        if not self.dry_run:
            frame.write(str(new_frame_path))

    def write_labels(self):
        if BACKGROUND_LABEL in self.labels:
            self.labels.remove(BACKGROUND_LABEL)
        labels_path = self.output_dir / "labels.txt"
        print("Writing labels to %s" % labels_path)
        self.write_list(labels_path, self.labels)

    def check_ratios(self, test_ratio, train_ratio, validation_ratio):
        return abs(1.0 - (test_ratio + train_ratio + validation_ratio)) < 1E-8

    def map_label_to_frames(self):
        label_unique_mapping = {}
        for index, frame in enumerate(self.frames):
            # use first object to characterize image for sorting
            if len(frame.objects) == 0:
                class_name = BACKGROUND_LABEL
            else:
                class_name = frame.objects[0].name
            if class_name not in label_unique_mapping:
                label_unique_mapping[class_name] = []
            label_unique_mapping[class_name].append(index)
        return label_unique_mapping

    def randomly_select(self, count, input_list, output_list):
        assert count <= len(input_list), f"{count} > {len(input_list)}"

        for _ in range(count):
            obj = random.choice(input_list)
            index = input_list.index(obj)
            output_list.append(input_list.pop(index))

    def get_annotation_idx(self, label_unique_mapping):
        image_indices = {
            "test": [],
            "train": [],
            "validation": []
        }
        for class_name, frame_indices in label_unique_mapping.items():
            class_count = len(frame_indices)
            counts = {
                "test": int(class_count * self.test_ratio),
                "train": int(class_count * self.train_ratio),
                "validation": int(class_count * self.validation_ratio)
            }

            for key, set_count in counts.items():
                self.randomly_select(set_count, frame_indices, image_indices[key])
            for _ in range(len(frame_indices)):  # dump any remaining frames into the training set
                image_indices["train"].append(frame_indices.pop())

            assert len(frame_indices) == 0, len(frame_indices)
            for key, indices in image_indices.items():
                if self.ratios[key] > 0.0:
                    assert len(indices) > 0, "%s is empty!" % key

            print(
                f"Label {class_name} count: {class_count}\n"
                f"\tTest: {counts['test']}\t{len(image_indices['test'])}\n"
                f"\tTrain: {counts['train']}\t{len(image_indices['train'])}\n"
                f"\tValidation: {counts['validation']}\t{len(image_indices['validation'])}"
            )
        return image_indices

    def frame_index_to_anno_path(self, index, relative=True):
        if relative:
            path = self.frames[index].frame_path
        else:
            path = os.path.abspath(self.frames[index].frame_path)
        return os.path.splitext(path)[0]

    def build_training_sets(self):
        imageset_dir = self.output_dir / IMAGESETS
        self.makedir(imageset_dir)

        set_paths = {
            "test": imageset_dir / "test.txt",
            "train": imageset_dir / "train.txt",
            "validation": imageset_dir / "val.txt",
            "trainval": imageset_dir / "trainval.txt",
        }

        label_unique_mapping = self.map_label_to_frames()
        image_indices = self.get_annotation_idx(label_unique_mapping)

        test_count = len(image_indices['test'])
        train_count = len(image_indices['train'])
        validation_count = len(image_indices['validation'])
        total_count = test_count + train_count + validation_count
        print(
            f"Total {total_count}:\n"
            f"\tTest: {test_count}\t{test_count / total_count:0.2f}\n"
            f"\tTrain: {train_count}\t{train_count / total_count:0.2f}\n"
            f"\tValidation: {validation_count}\t{validation_count / total_count:0.2f}"
        )

        set_anno_paths = {}
        for key, set_indices in image_indices.items():
            output_path = set_paths[key]
            anno_paths = [self.frame_index_to_anno_path(index, relative=True) for index in set_indices]
            set_anno_paths[key] = anno_paths
            print("Writing %s image set with %s annotations: %s" % (key, len(anno_paths), output_path))
            self.write_list(output_path, anno_paths)

        # trainval combines train and validation sets
        trainval_set = set_anno_paths["train"] + set_anno_paths["validation"]
        trainval_path = set_paths["trainval"]
        print("Writing trainval image set with %s annotations: %s" % (len(trainval_set), trainval_path))
        self.write_list(trainval_path, trainval_set)

    def write_list(self, path, l):
        if not self.dry_run:
            with open(path, 'w') as file:
                file.write("\n".join(l))
