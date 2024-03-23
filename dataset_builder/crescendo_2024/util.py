import csv
import json
import os
import sys

import yaml
from tj2_tools.training.get_image_size import get_image_size
from tj2_tools.training.yolo import YoloFrame, YoloObject


def get_best_model(base_dir):
    last_modified_time = 0.0
    last_modified_path = ""

    for dirname in os.listdir(base_dir):
        path = os.path.join(base_dir, dirname)
        modified_time = os.path.getmtime(path)
        if modified_time > last_modified_time:
            last_modified_time = modified_time
            last_modified_path = path
    return last_modified_path


def get_labels():
    return ["robot", "note"]


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


def load_validation_data(path, labels):
    table = {}
    if os.path.isfile(path):
        with open(path, "r") as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                frame_path = row[header.index("frame")]
                image_path = row[header.index("image")]
                if not os.path.isfile(frame_path):
                    print(f"{frame_path} is not a valid annotation. Removing from the table")
                    continue
                if not os.path.isfile(image_path):
                    print(f"{frame_path} is not a valid image. Removing from the table")
                    continue
                frame = YoloFrame.from_file(frame_path, image_path, labels)
                review = row[header.index("review")]
                table[frame] = (frame, review)
        print(f"Loaded {len(table)} rows")
    else:
        print("Not validation table found. Creating one.")
    return table


def save_validation_data(validation_table, path):
    with open(path, "w") as file:
        writer = csv.writer(file)
        writer.writerow(["frame", "image", "review"])
        for frame, review in validation_table.values():
            writer.writerow([frame.frame_path, frame.image_path, review])


def read_training_config(data_path):
    filename = os.path.basename(data_path)
    tmp_path = os.path.join("/tmp", filename)
    with open(os.path.join("/tmp", filename), "w") as tmp:
        with open(data_path, "r") as file:
            config = yaml.safe_load(file)
        base_dir = config["path"]
        config["path"] = os.path.abspath(os.path.join(base_dir, config.get("path", "")))
        config["train"] = os.path.abspath(os.path.join(base_dir, config.get("train", "")))
        config["val"] = os.path.abspath(os.path.join(base_dir, config.get("val", "")))
        config["test"] = os.path.abspath(os.path.join(base_dir, config.get("test", "")))
        labels = get_labels()
        config["nc"] = len(labels)
        config["names"] = labels
        yaml.dump(config, tmp)
    return config, tmp_path


def yolov5_module_path_hack():
    sys.path.insert(0, "/opt/yolov5")  # :/ yolov5 organizes its files badly
