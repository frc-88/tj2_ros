import cv2
import numpy as np
import random
from tj2_tools.training.detect_collector import DetectCollector
from tj2_tools.training.pascal_voc import PascalVOCFrame
from tj2_tools.training.helpers import debug_imshow


def pipeline(image, frame):
    blur_image = cv2.medianBlur(image, 15)
    hsv_image = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)
    thresholded = cv2.inRange(hsv_image, (15, 110, 100), (35, 255, 255))
    # bgr_threshold = cv2.cvtColor(thresholded, cv2.COLOR_GRAY2BGR)
    # return np.hstack((image, bgr_threshold))
    contours, hierarchy = cv2.findContours(image=thresholded, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(image=image, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2,
                     lineType=cv2.LINE_AA)
    return image


def walkthrough_images():
    random.seed(8888)
    base_dir = "resources/base_images"
    collector = DetectCollector(base_dir)

    for frame, image in collector.iter(include_image=True):
        frame = PascalVOCFrame.from_frame(frame)

        print(frame.path)
        output_image = pipeline(image, frame)
        debug_imshow(output_image, frame, 1080)


def main():
    walkthrough_images()


if __name__ == "__main__":
    main()
