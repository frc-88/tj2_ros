import os
from util import read_training_config, yolov5_module_path_hack
yolov5_module_path_hack()

import yolov5
import yolov5.train

filename = "charged_up_2023.yaml"
data_path = os.path.abspath(filename)

config, tmp_path = read_training_config(data_path)

model_path = os.path.join(os.path.dirname(yolov5.__file__), "models", "yolov5s.yaml")
yolov5.train.run(
    data=tmp_path,
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