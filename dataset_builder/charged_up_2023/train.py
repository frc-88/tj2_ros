import os
import sys
import yaml

import yolov5
import yolov5.train
from util import get_labels

filename = "charged_up_2023.yaml"
data_path = os.path.abspath(filename)

with open(os.path.join("/tmp", filename), 'w') as tmp:
    with open(data_path, 'r') as file:
        config = yaml.safe_load(file)
    base_dir = config["path"]
    config["path"] = os.path.abspath(os.path.join(base_dir, config.get("path", '')))
    config["train"] = os.path.abspath(os.path.join(base_dir, config.get("train", '')))
    config["val"] = os.path.abspath(os.path.join(base_dir, config.get("val", '')))
    config["test"] = os.path.abspath(os.path.join(base_dir, config.get("test", '')))
    labels = get_labels()
    config["nc"] = len(labels)
    config["names"] = labels
    yaml.dump(config, tmp)

    model_path = os.path.join(os.path.dirname(yolov5.__file__), "models", "yolov5s.yaml")
    yolov5.train.run(
        data=tmp.name,
        imgsz=640,
        epochs=500,
        cfg=model_path,
        weights='',
        device=0,
        batch_size=4,
        project=os.path.abspath("../data/outputs/charged_up_2023_train"),
        multi_scale=True,
        cache='ram',
        # resume='../data/outputs/charged_up_2023_train/exp11/weights/last.pt'
    )
