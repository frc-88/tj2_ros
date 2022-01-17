from tj2_tools.training.detect_collector import DetectCollector
from tj2_tools.training.dataset_builder.detect_dataset_builder import DetectDatasetBuilder
from pathlib import Path


def format_detect_dataset():
    base_dir = "outputs/frc_supervisely"

    collector = DetectCollector(base_dir)
    dataset = DetectDatasetBuilder(Path("outputs/cargo_2022_dataset"), collector, dry_run=False)
    dataset.reset()
    dataset.build()


if __name__ == '__main__':
    format_detect_dataset()
