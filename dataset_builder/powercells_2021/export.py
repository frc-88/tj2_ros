# https://docs.ultralytics.com/tutorials/torchscript-onnx-coreml-export/
import os

from yolov5 import export

export.run(
    data=os.path.abspath("outputs/powercell_2021.yaml"),
    weights=os.path.abspath("outputs/powercell_2021_train/exp/weights/best.pt"),
    device=0,
    imgsz=(640, 640),
)
