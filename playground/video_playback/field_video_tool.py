import cv2
import time


def main():
    window_name = "video"

    capture_path = "/home/benjamin/Videos/Qualification 35 - 2022 FIRST Championship - Galileo Division.mp4"

    cv2.namedWindow(window_name)

    frame_num = 0
    image = None
    capture = cv2.VideoCapture(capture_path)

    length = capture.get(cv2.CAP_PROP_FRAME_COUNT)
    fps = capture.get(cv2.CAP_PROP_FPS)

    def next_frame():
        nonlocal frame_num, image
        success, image = capture.read()
        frame_num += 1
        return success
    
    while True:
        if not next_frame():
            break
        cv2.imshow(window_name, image)
        value = cv2.waitKey(1)
        key = chr(value & 0xff)

        if key == 'q':
            break

if __name__ == '__main__':
    main()
