import os
import shutil
from util import get_labels, load_validation_data, save_validation_data
from tj2_tools.training.yolo import YoloFrame

def main():
    validation_path = "/root/tj2_ros/dataset_builder/data/charged_up_2023_raw/game_pieces/validation.csv"
    failed_path = "/root/tj2_ros/dataset_builder/data/charged_up_2023_raw/quarantine"
    
    labels = get_labels()
    validation_table = load_validation_data(validation_path, labels)
    
    
    for frame, review in validation_table.values():
        review = review.lower()
        if review == "fail" or review == "not reviewed":
            path = os.path.join(failed_path, os.path.basename(frame.frame_path))
            if not os.path.isfile(path):
                continue
            shutil.copy(path, frame.frame_path)
            validation_table[frame] = frame, "Not reviewed"
            print(f"Copied {path} to {frame.frame_path}. Marking as not reviewed")
    
    save_validation_data(validation_table, validation_path)

if __name__ == '__main__':
    main()
