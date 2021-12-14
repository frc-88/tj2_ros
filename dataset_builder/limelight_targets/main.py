import random
from pathlib import Path
from tj2_tools.training.pascal_voc import PascalVOCFrame

from image_collector import ImageCollector
from dataset_builder import DatasetBuilder
from pipelines import *
from helpers import record_annotation, debug_imshow


def generate_images():
    random.seed(8888)
    base_dir = "./base_images"
    collector = ImageCollector(base_dir)

    pipelines = {
        "a_with_gauss_background": (2, with_gauss_background),
        "b_with_gauss": (2, with_gauss),
        "c_with_colored_background": (2, with_colored_background),
        "d_move_objects_subtle": (25, move_objects_subtle),
        "e_move_objects_subtle": (25, move_objects_subtle_gauss_background),
        "f_move_objects_subtle_gauss": (25, move_objects_subtle_gauss),
        "g_false_objects_subtle": (10, false_objects_subtle),
        "h_false_objects_subtle_gauss_background": (5, false_objects_subtle_gauss_background),
        "i_false_objects_subtle_gauss": (5, false_objects_subtle_gauss),
        "j_move_objects_medium": (20, move_objects_medium),
        "k_move_objects_medium_gauss_background": (10, move_objects_medium_gauss_background),
        "l_move_objects_medium_gauss": (10, move_objects_medium_gauss),
        "m_move_objects_erratic": (10, move_objects_erratic),
        "n_move_objects_erratic_gauss_background": (5, move_objects_erratic_gauss_background),
        "o_move_objects_erratic_gauss": (5, move_objects_erratic_gauss),
        "p_window_false_objects": (40, window_false_objects),
    }

    for key, (iterations, pipeline) in pipelines.items():
        for _ in range(iterations):
            for frame, image in collector.iter(include_image=True):
                frame = PascalVOCFrame.from_frame(frame)

                output_image = pipeline(image, frame)
                # debug_imshow(output_image, frame)

                record_annotation(output_image, frame, "output_images", key)


def format_dataset():
    base_dir = "./output_images"

    collector = ImageCollector(base_dir)
    dataset = DatasetBuilder(Path("dataset"), collector, dry_run=False)
    dataset.build()

def try_pipeline():
    base_dir = "./base_images"
    collector = ImageCollector(base_dir)

    random.seed(8888)
    for frame, image in collector.iter(include_image=True):
        output_image = move_objects_subtle(image, frame)
        debug_imshow(output_image, frame)


def main():
    # try_pipeline()
    # generate_images()
    format_dataset()


if __name__ == "__main__":
    main()
