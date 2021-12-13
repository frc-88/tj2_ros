from tj2_tools.training.pascal_voc import PascalVOCFrame

from image_collector import ImageCollector
from pipelines import *
from helpers import record_annotation, debug_imshow


def main():
    base_dir = "./base_images"
    collector = ImageCollector(base_dir)

    pipelines = {
        "a_with_gauss_background": (20, with_gauss_background),
        "b_with_gauss": (20, with_gauss),
        "c_with_colored_background": (20, with_colored_background),
        "d_move_objects_subtle": (30, move_objects_subtle),
        "e_move_objects_subtle": (10, move_objects_subtle_gauss_background),
        "f_move_objects_subtle_gauss": (10, move_objects_subtle_gauss),
        "g_false_objects_subtle": (10, false_objects_subtle),
        "h_false_objects_subtle_gauss_background": (5, false_objects_subtle_gauss_background),
        "i_false_objects_subtle_gauss": (5, false_objects_subtle_gauss),
        "j_move_objects_medium": (5, move_objects_medium),
        "k_move_objects_medium_gauss_background": (5, move_objects_medium_gauss_background),
        "l_move_objects_medium_gauss": (5, move_objects_medium_gauss),
        "m_move_objects_erratic": (10, move_objects_erratic),
        "n_move_objects_erratic_gauss_background": (5, move_objects_erratic_gauss_background),
        "o_move_objects_erratic_gauss": (5, move_objects_erratic_gauss),
        "p_window_false_objects": (30, window_false_objects),
    }

    for key, (iterations, pipeline) in pipelines.items():
        for _ in range(iterations):
            for frame_collector, image in collector.iter(include_image=True):
                frame = PascalVOCFrame.from_frame(frame_collector)

                output_image = pipeline(image, frame)
                # debug_imshow(output_image, frame)

                record_annotation(output_image, frame, "output_images", key)


if __name__ == "__main__":
    main()
