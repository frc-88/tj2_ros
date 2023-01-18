from charged_up_2023_collector import ChargedUp2023Collector
from tj2_tools.training.dataset_builder.yolo_dataset_builder import YoloDatasetBuilder
from util import get_labels


def main():
    labels = get_labels()
    base_path = "/root/tj2_ros/dataset_builder/data/charged_up_2023_raw/game_pieces/video3"
    collector = ChargedUp2023Collector(base_path, labels)
    for frame in collector.iter():
        frame.write()

if __name__ == '__main__':
    main()
