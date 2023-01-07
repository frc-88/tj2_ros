import os
import cv2
import time
import numpy as np


def pipeline(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    contour_image = cv2.inRange(hsv_frame, (41, 135, 168), (88, 255, 255))
    contours, hierarchy = cv2.findContours(contour_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    bounding_boxes = [cv2.boundingRect(contour) for contour in contours]
    result_image = cv2.cvtColor(contour_image, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(result_image, contours, -1, (0, 255, 0), 1)
    for bndbox in bounding_boxes:
        x, y, w, h = bndbox
        cv2.rectangle(result_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
    return result_image


def main():
    cap = cv2.VideoCapture("http://10.0.88.42:5802/")

    prev_log_time = time.time()
    read_times = []
    loop_times = []
    window_size = 20
    resize_width = 1920

    try:
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            t1 = time.time()

            result = pipeline(frame)
            if result is None:
                draw_frame = frame
            else:
                draw_frame = result

            height, width = draw_frame.shape[0:2]
            resize_height = int(resize_width / width * height)
            draw_frame = cv2.resize(draw_frame, (resize_width, resize_height))

            cv2.imshow("Capturing", draw_frame)

            key = chr(cv2.waitKey(1) & 0xff)

            if key == "q":
                break
            t2 = time.time()

            read_times.append(t1 - t0)
            loop_times.append(t2 - t0)
            while len(read_times) > window_size:
                read_times.pop(0)
            while len(loop_times) > window_size:
                loop_times.pop(0)

            if time.time() - prev_log_time > 1.0:
                prev_log_time = time.time()
                print(
                    "Read: %0.2f, Loop: %0.2f" % (len(read_times) / sum(read_times), len(loop_times) / sum(loop_times)))
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
