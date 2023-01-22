import os
import cv2
import tqdm
import numpy as np
import matplotlib
from matplotlib import pyplot as plt

from tj2_tools.training.yolo import YoloFrame
from charged_up_2023_collector import ChargedUp2023Collector
from util import get_labels, load_validation_data, save_validation_data

matplotlib.use("TkAgg")


class Validator:
    def __init__(self) -> None:
        self.labels = get_labels()
        base_path = "/root/tj2_ros/dataset_builder/data/charged_up_2023_raw/game_pieces"
        self.output_path = "/root/tj2_ros/dataset_builder/data/charged_up_2023_raw/game_pieces/validation.csv"
        self.frames = []
        self.collector = ChargedUp2023Collector(base_path, self.labels)
        self.pbar = tqdm.tqdm(desc="image", total=self.collector.get_length(), bar_format='Loading: {desc}{percentage:3.0f}%|{bar}|{n}/{total}')
        self.generator = self.collector.iter()
        self.current_index = 0
        
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect('key_press_event', lambda event: self.on_press(event))
        
        self.colors = {
            "cone": [0.863, 0.682, 0.2],
            "cube": [0.278, 0.227, 0.643],
        }
        
        self.validation_table = load_validation_data(self.output_path, self.labels)

    def on_press(self, event):
        index = self.current_index
        review = ""
        if event.key == "left":
            index -= 1
        elif event.key == "right":
            index += 1
        elif event.key == "a":
            index = self.get_next_unvalidated_index(index - 1)
        elif event.key == "d":
            index = self.get_next_unvalidated_index(index + 1)
        elif event.key == "g":
            review = self.validate_frame(index, "Pass")
            index += 1
        elif event.key == "b":
            review = self.validate_frame(index, "Fail")
            index += 1
        elif event.key == "n":
            review = self.validate_frame(index, "Not reviewed")
        elif event.key == "q":
            quit()
        else:
            print("pressed", event.key)
        index = self.bound_index(index)
        if index != self.current_index or len(review) != 0:
            self.draw_frame(index)
            self.pbar.update(index - self.current_index)
            self.pbar.refresh()
            self.current_index = index
    
    def bound_index(self, index: int) -> int:
        return max(0, min(self.collector.get_length() - 1, index))
    
    def get_next_unvalidated_index(self, index: int) -> int:
        increment = -1 if index - self.current_index < 0 else 1
        check_index = index
        not_reviewed = False
        while not not_reviewed:
            check_index += increment
            if check_index < 0 or check_index >= self.collector.get_length():
                break
            frame = self.load_index(check_index)
            if self.is_reviewed(frame):
                not_reviewed = False
            else:
                not_reviewed = True
        return self.bound_index(check_index)
    
    def is_reviewed(self, frame: YoloFrame) -> bool:
        return frame in self.validation_table and self.validation_table[frame][1] != "Not reviewed"
    
    def validate_frame(self, index, review: str) -> str:
        frame = self.frames[index]
        self.validation_table[frame] = frame, review
        print(f"Marking {frame.frame_path} as {review}")
        save_validation_data(self.validation_table, self.output_path)
        return review

    def load_index(self, index) -> YoloFrame:
        try:
            while index >= len(self.frames):
                print(f"Loading index {len(self.frames)}")
                self.frames.append(next(self.generator))
        except StopIteration:
            pass
        return self.frames[index]
    
    def draw_frame(self, index):
        self.ax.clear()
        frame = self.load_index(index)
        image = cv2.imread(frame.image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.ax.imshow(image)

        review_text = self.validation_table[frame][1] if self.is_reviewed(frame) else "Not reviewed"
        self.ax.set_title(f"{os.path.basename(frame.frame_path)}: {review_text}")
        
        for obj in frame.objects:
            self.ax.annotate(
                obj.label, xy=(obj.bounding_box.x0, obj.bounding_box.y0),
                arrowprops=dict(facecolor='black', shrink=0.05),
                horizontalalignment='right', verticalalignment='top',
            )
            bb_x0, bb_y0, bb_x1, bb_y1 = obj.bounding_box.x0, obj.bounding_box.y0, obj.bounding_box.x1, obj.bounding_box.y1
            if obj.label not in self.colors:
                color = (np.random.random((1, 3))*0.8+0.2).tolist()[0]
            else:
                color = self.colors[obj.label]
            
            self.ax.plot([bb_x0, bb_x0, bb_x1, bb_x1, bb_x0], [bb_y0, bb_y1, bb_y1, bb_y0, bb_y0], '-', color=color)
        self.fig.canvas.draw()
    
    def run(self):
        self.draw_frame(0)
        plt.show()


def main():
    validator = Validator()
    validator.run()

if __name__ == '__main__':
    main()
