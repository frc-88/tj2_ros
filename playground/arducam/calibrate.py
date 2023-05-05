import cv2
import argparse

def main():
    parser = argparse.ArgumentParser("calibrate")
    parser.add_argument("board-width", type=int)
    parser.add_argument("board-height", type=int)
    args = parser.parse_args()
    
    # Define the dimensions of checkerboard
    checkerboard = (args.board_width, args.board_height)

    # stop the iteration when specified
    # accuracy, epsilon, is reached or
    # specified number of iterations are completed.
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
