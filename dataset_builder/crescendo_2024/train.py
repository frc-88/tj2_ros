import os

from util import read_training_config, yolov5_module_path_hack

yolov5_module_path_hack()

import yolov5
import yolov5.models
import yolov5.train

filename = "crescendo_2024.yaml"
data_path = os.path.abspath(filename)

config, tmp_path = read_training_config(data_path)

models_dir = os.path.dirname(yolov5.models.__file__)

model_path = os.path.join(models_dir, "yolov5s.yaml")
yolov5.train.run(
    data=tmp_path,
    imgsz=640,
    # epochs=500,
    epochs=250,
    cfg=model_path,
    weights="",
    device=0,
    batch_size=4,
    project=os.path.abspath("../data/outputs/crescendo_2024_train"),
    multi_scale=True,
    cache="ram",
    resume="../data/outputs/crescendo_2024_train/exp3/weights/last.pt",
)
