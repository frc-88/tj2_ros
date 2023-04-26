import os
import shutil
from util import get_labels, load_validation_data

def main():
    validation_path = "/home/tj2/tj2_ros/dataset_builder/data/charged_up_2023_raw/game_pieces/validation.csv"
    failed_path = "/home/tj2/tj2_ros/dataset_builder/data/charged_up_2023_raw/quarantine"
    if not os.path.isdir(failed_path):
        os.makedirs(failed_path)
    
    validation_table = load_validation_data(validation_path, get_labels())
    
    for frame, review in validation_table.values():
        review = review.lower()
        if review == "fail":
            print(f"Copied {frame.frame_path} to {failed_path}")
            shutil.copy(frame.frame_path, failed_path)
            shutil.copy(frame.image_path, failed_path)

if __name__ == '__main__':
    main()
