# https://docs.ultralytics.com/tutorials/torchscript-onnx-coreml-export/
import datetime
import os
from pathlib import Path

from util import get_best_model, read_training_config, yolov5_module_path_hack

yolov5_module_path_hack()

import yolov5
import yolov5.export

model_name = datetime.datetime.now().strftime("crescendo_%Y-%m-%d.torchscript")
classes_name = "crescendo_2024.names"
config_name = "crescendo_2024.yaml"
config_path = os.path.abspath(config_name)

base_dir = "../data/outputs/crescendo_2024_train"

last_modified_path = get_best_model(base_dir)
config, tmp_path = read_training_config(config_path)

weights_dir = os.path.abspath(os.path.join(last_modified_path, "weights"))
data_path = os.path.abspath("crescendo_2024.yaml")

yolov5.export.run(
    data=Path(data_path),
    weights=Path(weights_dir) / "best.pt",
    device=0,
    imgsz=(640, 640),
)

old_torchscript = os.path.join(weights_dir, "best.torchscript")
new_torchscript = os.path.join(weights_dir, model_name)
os.rename(old_torchscript, new_torchscript)

names_path = os.path.join(weights_dir, classes_name)

classes = config["names"]

with open(names_path, "w") as file:
    file.write("\n".join(classes))


print(f"Model exported to {new_torchscript}")
print(f"Names written to {names_path}")
