import os
import shutil
import random
from pathlib import Path
from tj2_tools.training.pascal_voc import PascalVOCFrame

from tj2_tools.training.detect_collector import DetectCollector
from tj2_tools.training.dataset_builder.detect_dataset_builder import DetectDatasetBuilder
from tj2_tools.training.dataset_builder.classify_dataset_builder import ClassifyDatasetBuilder
from pipelines import *
from tj2_tools.training.helpers import record_annotation, debug_imshow, crop_to_annotations, record_image, record_classify_paths, \
    read_classify_paths, crop_to_background


def generate_detect_images():
    random.seed(8888)
    base_dir = "resources/base_images"
    collector = DetectCollector(base_dir)

    pipelines = {
        "00_original": (1, no_operation),
        "a_with_gauss_background": (4, with_gauss_background),
        "b_with_gauss": (3, with_gauss),
        "c_with_colored_background": (2, with_colored_background),
        "d_move_objects_subtle": (10, move_objects_subtle),
        "e_move_objects_subtle": (10, move_objects_subtle_gauss_background),
        "f_move_objects_subtle_gauss": (10, move_objects_subtle_gauss),
        "g_false_objects_subtle": (10, false_objects_subtle),
        "h_false_objects_subtle_gauss_background": (5, false_objects_subtle_gauss_background),
        "i_false_objects_subtle_gauss": (5, false_objects_subtle_gauss),
        "j_move_objects_medium": (5, move_objects_medium),
        "k_move_objects_medium_gauss_background": (5, move_objects_medium_gauss_background),
        "l_move_objects_medium_gauss": (5, move_objects_medium_gauss),
        "m_move_objects_erratic": (5, move_objects_erratic),
        "n_move_objects_erratic_gauss_background": (5, move_objects_erratic_gauss_background),
        "o_move_objects_erratic_gauss": (5, move_objects_erratic_gauss),
        "p_window_false_objects": (20, window_false_objects),
    }

    output_dir = "outputs/output_detect_images"
    if os.path.isdir(output_dir):
        shutil.rmtree(output_dir)

    count = 0
    for key, (iterations, pipeline) in pipelines.items():
        for _ in range(iterations):
            for frame, image in collector.iter(include_image=True):
                frame = PascalVOCFrame.from_frame(frame)

                output_image = pipeline(image, frame)
                # debug_imshow(output_image, frame)

                record_annotation(output_image, frame, output_dir, key)
                count += 1
    print("Wrote %s images" % count)


def generate_classify_images():
    random.seed(8888)
    base_dir = "resources/base_images"
    collector = DetectCollector(base_dir)

    pipelines = {
        "00_original": (1, no_operation),
        "a_with_gauss_background": (1, with_gauss_background),
        "b_with_gauss": (1, with_gauss),
        "c_with_colored_background": (3, with_colored_background),
        "d_move_objects_subtle": (1, move_objects_subtle),
        "e_move_objects_subtle": (1, move_objects_subtle_gauss_background),
        "f_move_objects_subtle_gauss": (1, move_objects_subtle_gauss),
        "g_false_objects_subtle": (1, false_objects_subtle),
        "h_false_objects_subtle_gauss_background": (1, false_objects_subtle_gauss_background),
        "i_false_objects_subtle_gauss": (1, false_objects_subtle_gauss),
        "j_move_objects_medium": (2, move_objects_medium),
        "k_move_objects_medium_gauss_background": (1, move_objects_medium_gauss_background),
        "l_move_objects_medium_gauss": (1, move_objects_medium_gauss),
        "m_move_objects_erratic": (1, move_objects_erratic),
        "n_move_objects_erratic_gauss_background": (1, move_objects_erratic_gauss_background),
        "o_move_objects_erratic_gauss": (1, move_objects_erratic_gauss),
        "p_window_false_objects": (5, window_false_objects),
    }
    output_dir = "outputs/output_classify_images"
    if os.path.isdir(output_dir):
        shutil.rmtree(output_dir)

    image_paths = {}

    count = 0
    for key, (iterations, pipeline) in pipelines.items():
        for _ in range(iterations):
            for frame, image in collector.iter(include_image=True):
                frame = PascalVOCFrame.from_frame(frame)

                output_image = pipeline(image, frame)
                crops = crop_to_annotations(output_image, frame)
                backgrounds = crop_to_background(output_image, frame, (20, 20), (60, 60), 2)

                crops.update(backgrounds)
                for label, images in crops.items():
                    for crop_image in images:
                        # debug_imshow(crop_image, frame)
                        prefix = key + "_" + label
                        image_path = record_image(crop_image, output_dir, resize=(300, 300), prefix=prefix)

                        if label not in image_paths:
                            image_paths[label] = []
                        image_paths[label].append(os.path.basename(image_path))
                        count += 1
    print("Wrote %s images" % count)
    record_classify_paths(image_paths, output_dir)


def format_detect_dataset():
    base_dir = "outputs/output_detect_images"

    collector = DetectCollector(base_dir)
    dataset = DetectDatasetBuilder(Path("outputs/detect_dataset"), collector, dry_run=False)
    dataset.reset()
    dataset.build()


def format_classify_dataset():
    base_dir = Path("outputs/output_classify_images")
    image_paths = read_classify_paths(base_dir)
    dataset = ClassifyDatasetBuilder(
        Path("outputs/classify_dataset"), base_dir, image_paths,
        test_ratio=0.15, train_ratio=0.8, validation_ratio=0.05,
        dry_run=False)
    dataset.reset()
    dataset.build()


def try_pipeline():
    base_dir = "resources/base_images"
    collector = DetectCollector(base_dir)

    random.seed(8888)
    for frame, image in collector.iter(include_image=True):
        output_image = move_objects_subtle(image, frame)
        debug_imshow(output_image, frame)


def main():
    # try_pipeline()
    # generate_detect_images()
    # format_detect_dataset()
    generate_classify_images()
    format_classify_dataset()


if __name__ == "__main__":
    main()
