
import os
import cv2
import tqdm
import argparse
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from cv_bridge import CvBridge
from tj2_tools.yolo.detector import YoloDetector
from tj2_tools.rosbag_to_file.utils import enumerate_bag, Options, get_bag_length
from util import get_labels, get_best_model, yolov5_module_path_hack

yolov5_module_path_hack()
matplotlib.use("TkAgg")


class Tester:
    def __init__(self, bag_path) -> None:
        self.labels = get_labels()
        self.frames = []
        self.current_index = 0
        self.bridge = CvBridge()
        options = Options(bag_path, "")
        self.bag_generator = enumerate_bag(options)
        self.bag_length = get_bag_length(options)
        self.pbar = tqdm.tqdm(desc="image", total=self.bag_length, bar_format='Loading: {desc}{percentage:3.0f}%|{bar}|{n}/{total}')
        
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect('key_press_event', lambda event: self.on_press(event))
        
        self.colors = {
            "cone": [0.863, 0.682, 0.2],
            "cube": [0.278, 0.227, 0.643],
        }
        self.show_truth = True
        
        model_device = 0
        model_dir = get_best_model("/opt/tj2/tj2_ros/dataset_builder/data/outputs/charged_up_2023_train")
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
            model_device, model_path, self.image_width, self.image_height,
            confidence_threshold, nms_iou_threshold, max_detections,
            report_loop_times, publish_overlay
        )

    def on_press(self, event):
        index = self.current_index
        redraw = False
        if event.key == "left":
            index -= 1
        elif event.key == "right":
            index += 1
        elif event.key == "q":
            quit()
        elif event.key == "t":
            self.show_truth = not self.show_truth
            redraw = True
        else:
            print("pressed", event.key)
        if index != self.current_index or redraw:
            self.draw_frame(self.load_index(index))
            self.pbar.update(index - self.current_index)
            self.pbar.refresh()
            self.current_index = index
    
    def bound_index(self, index: int) -> int:
        return max(0, min(len(self.frames) - 1, index))

    def load_index(self, index) -> np.ndarray:
        while index >= len(self.frames):
            frame = self.next_image()
            if frame is None:
                break
            self.frames.append(frame)
        index = self.bound_index(index)
        return self.frames[index]
    
    def next_image(self):
        try:
            while True:
                topic, msg, timestamp = next(self.bag_generator)
                if topic.startswith("/tj2_zed/rgb/") and "image" in str(type(msg)).lower():
                    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    return image
        except StopIteration:
            pass
        return None

    def draw_frame(self, image):
        self.ax.clear()
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.ax.imshow(image_rgb)

        detections, overlay_image = self.yolo.detect(image)

        # overlay_image = cv2.cvtColor(overlay_image, cv2.COLOR_BGR2RGB)
        # self.ax.imshow(overlay_image)

        self.ax.set_title("Detection")

        print("Num detections:", len(detections))
        for obj_id, (bndbox, confidence) in detections.items():
            label, count = self.yolo.get_label(obj_id)
            if label not in self.colors:
                color = (np.random.random((1, 3))*0.8+0.2).tolist()[0]
            else:
                color = self.colors[label]
            bb_x0, bb_y0, bb_x1, bb_y1 = bndbox
            self.ax.plot([bb_x0, bb_x0, bb_x1, bb_x1, bb_x0], [bb_y0, bb_y1, bb_y1, bb_y0, bb_y0], '-', color=color)
            self.ax.annotate(
                f"{label}-{count}|{confidence * 100.0:0.1f}", xy=(bb_x1, bb_y1),
                color='k',
                arrowprops=dict(facecolor='k', shrink=0.05),
                horizontalalignment='right', verticalalignment='bottom',
            )
        
        print(self.yolo.timing_report)
        self.fig.canvas.draw()
    
    def run(self):
        self.draw_frame(self.load_index(0))
        plt.show()
        print("Finished")


def main():
    parser = argparse.ArgumentParser(description="test_on_bag", add_help=True)
    parser.add_argument("bag_path", help="path to bag to test")
    args = parser.parse_args()
    
    tester = Tester(args.bag_path)
    tester.run()

if __name__ == '__main__':
    main()
