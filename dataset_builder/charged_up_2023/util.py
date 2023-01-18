import os
import csv
import json
from tj2_tools.training.yolo import YoloFrame, YoloObject
from tj2_tools.training.get_image_size import get_image_size


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
    return ["cone", "cube"]

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
        with open(path, 'r') as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                frame_path = row[header.index("frame")]
                image_path = row[header.index("image")]
                frame = YoloFrame.from_file(frame_path, image_path, labels)
                review = row[header.index("review")]
                table[frame] = (frame, review)
        print(f"Loaded {len(table)} rows")
    else:
        print("Not validation table found. Creating one.")
    return table

def save_validation_data(validation_table, path):
    with open(path, 'w') as file:
        writer = csv.writer(file)
        writer.writerow(["frame", "image", "review"])
        for frame, review in validation_table.values():
            writer.writerow([frame.frame_path, frame.image_path, review])
