import random
from charged_up_2023_collector import ChargedUp2023Collector
from tj2_tools.training.dataset_builder.yolo_dataset_builder import YoloDatasetBuilder
from util import get_labels


def main():
    random.seed(2023)
    labels = get_labels()
    base_path = "/opt/tj2/tj2_ros/dataset_builder/data/charged_up_2023_raw/game_pieces"
    output_path = "/opt/tj2/tj2_ros/dataset_builder/data/charged_up_2023/game_pieces"
    collector = ChargedUp2023Collector(base_path, labels)
    builder = YoloDatasetBuilder(
        output_path, collector, labels,
        test_ratio=0.01, train_ratio=0.9, validation_ratio=0.09,
        dry_run=False
    )
    builder.build()

if __name__ == '__main__':
    main()
