import argparse
import os
import time

import cv2
import numpy as np
import tqdm
from cv_bridge import CvBridge
from tj2_tools.rosbag_to_file.utils import Options, enumerate_bag, get_bag_length
from util import get_best_model, get_labels, yolov5_module_path_hack

yolov5_module_path_hack()

from tj2_tools.yolo.detector import YoloDetector


class Player:
    def __init__(self, path) -> None:
        self.labels = get_labels()
        self.frames = []
        self.current_index = 0
        self.bridge = CvBridge()
        if path.endswith(".bag"):
            options = Options(path, "")
            self.image_generator = self.bag_generator(options)
            self.length = get_bag_length(options)
        else:
            video = cv2.VideoCapture(path)
            self.length = video.get(cv2.CAP_PROP_FRAME_COUNT)
            self.image_generator = self.video_generator(video)
        self.pbar = tqdm.tqdm(
            desc="image", total=self.length, bar_format="Loading: {desc}{percentage:3.0f}%|{bar}|{n}/{total}"
        )

        self.colors = {
            "robot": [1.0, 0.0, 0.0],
            "note": [0.94, 0.5, 0.42],
        }
        self.show_truth = True

        model_device = 0
        model_dir = get_best_model("/opt/tj2/tj2_ros/dataset_builder/data/outputs/crescendo_2024_train")
        model_path = os.path.join(model_dir, "weights/best.pt")
        print("Model:", model_path)
        self.image_width = 1280
        self.image_height = 720
        confidence_threshold = 0.5
        nms_iou_threshold = 0.4
        max_detections = 10
        report_loop_times = True
        publish_overlay = True
        self.yolo = YoloDetector(
            model_device,
            model_path,
            self.image_width,
            self.image_height,
            confidence_threshold,
            nms_iou_threshold,
            max_detections,
            report_loop_times,
            publish_overlay,
        )
        self.num_images = None
        self.window_name = os.path.basename(model_path)
        cv2.namedWindow(self.window_name)

    def bag_generator(self, options):
        for topic, msg, timestamp in enumerate_bag(options):
            if "image" in str(type(msg)).lower():
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                yield image

    def video_generator(self, video):
        success = True
        while success:
            success, image = video.read()
            yield image

    def bound_index(self, index: int) -> int:
        return max(0, min(len(self.frames) - 1, index))

    def load_index(self, index) -> np.ndarray:
        while index >= len(self.frames):
            frame = self.next_image()
            if frame is None:
                self.num_images = len(self.frames)
                break
            self.frames.append(frame)
        index = self.bound_index(index)
        return self.frames[index]

    def next_image(self):
        try:
            while True:
                return next(self.image_generator)
        except StopIteration:
            pass
        return None

    def draw_frame(self, image):
        detections, overlay_image = self.yolo.detect(image)
        cv2.imshow(self.window_name, overlay_image)
        print(self.yolo.timing_report)

    def run(self):
        prev_time = time.time()
        while True:
            self.draw_frame(self.load_index(self.current_index))

            key_value = cv2.waitKey(1)
            key = chr(key_value & 0xFF)
            if key == "q":
                quit()
            self.current_index += 1
            if self.num_images is not None and self.current_index >= self.num_images:
                self.current_index = 0
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            print("%0.4f fps" % (1.0 / dt))


def main():
    parser = argparse.ArgumentParser(description="play_on_bag", add_help=True)
    parser.add_argument("path", help="path to test")
    args = parser.parse_args()

    player = Player(args.path)
    player.run()


if __name__ == "__main__":
    main()
