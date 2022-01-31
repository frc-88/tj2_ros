import sys
from pathlib import Path

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.training.detect_collector import DetectCollector
from tj2_tools.training.dataset_builder.yolo_dataset_builder import YoloDatasetBuilder


def format_detect_dataset():
    base_dir = "resources/base_images"

    collector = DetectCollector(base_dir)
    dataset = YoloDatasetBuilder(Path("outputs/powercell_2021_yolo_dataset"), collector, ["power_cell"],
                                 dry_run=False)
    dataset.reset()
    dataset.build()


if __name__ == '__main__':
    format_detect_dataset()
