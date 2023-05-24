import os
import cv2
import tqdm
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from tj2_tools.training.yolo import YoloFrame
from charged_up_2023_collector import ChargedUp2023Collector
from util import get_labels, get_best_model, yolov5_module_path_hack

yolov5_module_path_hack()

from tj2_tools.yolo.detector import YoloDetector

matplotlib.use("TkAgg")


class Tester:
    def __init__(self) -> None:
        self.labels = get_labels()
        annotations_path = "/opt/tj2/tj2_ros/dataset_builder/data/charged_up_2023/game_pieces/labels/test"
        images_path = "/opt/tj2/tj2_ros/dataset_builder/data/charged_up_2023/game_pieces/images/test"
        self.frames = []
        self.collector = ChargedUp2023Collector(annotations_path, self.labels, images_path)
        self.pbar = tqdm.tqdm(desc="image", total=self.collector.get_length(), bar_format='Loading: {desc}{percentage:3.0f}%|{bar}|{n}/{total}')
        self.generator = self.collector.iter()
        self.current_index = 0
        
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect('key_press_event', lambda event: self.on_press(event))
        
        self.colors = {
            "cone": [0.863, 0.682, 0.2],
            "cube": [0.278, 0.227, 0.643],
        }
        self.highlight_colors = {
            "truth": [0.0, 0.5, 0.0],
            "inference": [1.0, 0.0, 0.0],
        }
        self.show_truth = True
        
        model_device = 0
        model_dir = get_best_model("/opt/tj2/tj2_ros/dataset_builder/data/outputs/charged_up_2023_train")
        model_path = os.path.join(model_dir, "weights/best.pt")
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
        index = self.bound_index(index)
        if index != self.current_index or redraw:
            self.draw_frame(index)
            self.pbar.update(index - self.current_index)
            self.pbar.refresh()
            self.current_index = index
    
    def bound_index(self, index: int) -> int:
        return max(0, min(self.collector.get_length(), index))

    def load_index(self, index) -> YoloFrame:
        while index >= len(self.frames):
            self.frames.append(next(self.generator))
        return self.frames[index]
    
    def draw_frame(self, index):
        self.ax.clear()
        frame = self.load_index(index)
        image_bgr = cv2.imread(frame.image_path)
        image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        self.ax.imshow(image)

        detections, overlay_image = self.yolo.detect(image_bgr)

        # overlay_image = cv2.cvtColor(overlay_image, cv2.COLOR_BGR2RGB)
        # self.ax.imshow(overlay_image)

        self.ax.set_title(f"{os.path.basename(frame.frame_path)}")

        if self.show_truth:
            for obj in frame.objects:
                bb_x0, bb_y0, bb_x1, bb_y1 = obj.bounding_box.x0, obj.bounding_box.y0, obj.bounding_box.x1, obj.bounding_box.y1
                if obj.label not in self.colors:
                    color = (np.random.random((1, 3))*0.8+0.2).tolist()[0]
                else:
                    color = self.colors[obj.label]
                highlight = self.highlight_colors["truth"]
                
                self.ax.plot([bb_x0, bb_x0, bb_x1, bb_x1, bb_x0], [bb_y0, bb_y1, bb_y1, bb_y0, bb_y0], '-', color=highlight, linewidth=5)
                self.ax.plot([bb_x0, bb_x0, bb_x1, bb_x1, bb_x0], [bb_y0, bb_y1, bb_y1, bb_y0, bb_y0], '-', color=color)
                self.ax.annotate(
                    obj.label, xy=(obj.bounding_box.x0, obj.bounding_box.y0),
                    color=highlight,
                    arrowprops=dict(facecolor=highlight, shrink=0.05),
                    horizontalalignment='left', verticalalignment='top',
                )
        
        print("Num detections:", len(detections))
        for obj_id, (bndbox, confidence) in detections.items():
            label, count = self.yolo.get_label(obj_id)
            if label not in self.colors:
                color = (np.random.random((1, 3))*0.8+0.2).tolist()[0]
            else:
                color = self.colors[label]
            highlight = self.highlight_colors["inference"]
            bb_x0, bb_y0, bb_x1, bb_y1 = bndbox
            self.ax.plot([bb_x0, bb_x0, bb_x1, bb_x1, bb_x0], [bb_y0, bb_y1, bb_y1, bb_y0, bb_y0], '-', color=highlight, linewidth=5)
            self.ax.plot([bb_x0, bb_x0, bb_x1, bb_x1, bb_x0], [bb_y0, bb_y1, bb_y1, bb_y0, bb_y0], '-', color=color)
            self.ax.annotate(
                f"{label}-{count}|{confidence * 100.0:0.1f}", xy=(bb_x1, bb_y1),
                color=highlight,
                arrowprops=dict(facecolor=highlight, shrink=0.05),
                horizontalalignment='right', verticalalignment='bottom',
            )
        
        print(self.yolo.timing_report)
        self.fig.canvas.draw()
    
    def run(self):
        self.draw_frame(0)
        plt.show()
        print("Finished")


def main():
    tester = Tester()
    tester.run()

if __name__ == '__main__':
    main()
