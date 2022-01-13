import os
import cv2
import numpy as np
import shutil
import random
from tj2_tools.training.detect_collector import DetectCollector
from tj2_tools.training.pascal_voc import PascalVOCFrame

from tj2_tools.training.helpers import crop_to_annotations, blank_background, apply_objects_to_background, \
    record_image, record_classify_paths, debug_imshow, pad_bndbox


def get_object_mask(bndbox, image):
    # annotated_image = image[bndbox[1]:bndbox[3], bndbox[0]:bndbox[2]]  # crop to annotation ROI
    # annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2HSV)
    # thresholded = cv2.inRange(annotated_image, (20, 0, 0), (35, 255, 255))
    # obj_mask = np.zeros(image.shape[0:2], dtype=np.uint8)
    # obj_mask[bndbox[1]:bndbox[3], bndbox[0]:bndbox[2]] = thresholded

    obj_mask = np.zeros(image.shape[0:2], dtype=np.uint8)
    obj_mask[bndbox[1]:bndbox[3], bndbox[0]:bndbox[2]] = 255

    return obj_mask


subtle_warps = dict(
    random_width=10,
    lower_ratio=0.2, upper_ratio=10.0,
    random_angle=0.1 * np.pi,
    min_percentage=0.9, max_percentage=10.0,
    obj_max_count=dict(power_cell=10)
)

no_warps = dict(
    random_width=0,
    lower_ratio=0.0, upper_ratio=0.0,
    random_angle=0.0,
    min_percentage=0.0, max_percentage=0.0,
    obj_max_count=dict(power_cell=10)
)


def pipeline(image, frame):
    # background = blank_background(image.shape)
    background = image
    return apply_objects_to_background(image, background, frame, get_object_mask, no_warps)


def remove_small_boxes(frame: PascalVOCFrame, min_width, min_height):
    remove_indices = []
    for index, obj in enumerate(frame.objects):
        xmin, ymin, xmax, ymax = obj.bndbox
        if xmax - xmin < min_width:
            remove_indices.append(index)
            continue
        if ymax - ymin < min_height:
            remove_indices.append(index)
            continue
    for index in sorted(remove_indices, reverse=True):
        frame.objects.pop(index)
    return frame


def generate_classify_images():
    random.seed(8888)
    base_dir = "resources/base_images"
    collector = DetectCollector(base_dir)

    output_dir = "outputs/output_classify_images"
    if os.path.isdir(output_dir):
        shutil.rmtree(output_dir)

    image_paths = {}

    tmp_count = 0
    count = 0
    for frame, image in collector.iter(include_image=True):
        frame = PascalVOCFrame.from_frame(frame)

        remove_small_boxes(frame, 30, 30)

        for obj in frame.objects:
            pad_bndbox(obj, 20, 20, image.shape[1], image.shape[0])
        print(frame.path)
        output_image = pipeline(image, frame)
        crops = crop_to_annotations(output_image, frame)
        # debug_imshow(output_image, frame)

        for label, images in crops.items():
            for crop_image in images:
                image_path = record_image(crop_image, output_dir, resize=(300, 300), prefix=label)

                if label not in image_paths:
                    image_paths[label] = []
                image_paths[label].append(os.path.basename(image_path))
                count += 1
        tmp_count += 1
        # if tmp_count > 15:
        #     break
    print("Wrote %s images" % count)
    record_classify_paths(image_paths, output_dir)


def main():
    generate_classify_images()


if __name__ == "__main__":
    main()
