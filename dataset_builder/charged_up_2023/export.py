# https://docs.ultralytics.com/tutorials/torchscript-onnx-coreml-export/
import os
import sys
import yaml

from util import get_best_model

import yolov5
import yolov5.export

model_name = "charged_up_2023-01-17.torchscript"
classes_name = "charged_up_2023.names"

base_dir = "../data/outputs/charged_up_2023_train"

last_modified_path = get_best_model(base_dir)

weights_dir = os.path.abspath(os.path.join(last_modified_path, "weights"))
data_path = os.path.abspath("charged_up_2023.yaml")

yolov5.export.run(
    data=data_path,
    weights=os.path.join(weights_dir, "best.pt"),
    device=0,
    imgsz=(640, 640),
)

old_torchscript = os.path.join(weights_dir, "best.torchscript")
new_torchscript = os.path.join(weights_dir, model_name)
os.rename(old_torchscript, new_torchscript)

names_path = os.path.join(weights_dir, classes_name)

with open(data_path) as file:
    config = yaml.safe_load(file)
    classes = config["names"]

with open(names_path, 'w') as file:
    file.write("\n".join(classes))


print(f"Model exported to {new_torchscript}")
print(f"Names written to {names_path}")
