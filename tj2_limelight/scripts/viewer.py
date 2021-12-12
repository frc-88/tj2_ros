import os
import cv2
import time
import numpy as np


CHECKERBOARD = (8, 5)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 28, 0.001)


def record_image(image, directory):
    if not os.path.isdir(directory):
        os.makedirs(directory)

    count = 0
    path = None
    while path is None:
        path = os.path.join(directory, "image-%04d.jpg" % count)
        if os.path.isfile(path):
            path = None
            count += 1
    print("Writing to %s" % path)
    cv2.imwrite(path, image)


def draw_checkerboard(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ret:
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)

        # Draw and display the corners
        return cv2.drawChessboardCorners(image, CHECKERBOARD, corners2, ret)
    else:
        return None

def main():
    cap = cv2.VideoCapture("http://10.0.88.42:5802/")

    prev_log_time = time.time()
    read_times = []
    loop_times = []
    window_size = 20
    resize_width = 1920

    while True:
        t0 = time.time()
        ret, frame = cap.read()
        t1 = time.time()

        result = draw_checkerboard(frame)
        if result is not None:
            frame = result

        height, width = frame.shape[0:2]
        resize_height = int(resize_width / width * height)
        frame = cv2.resize(frame, (resize_width, resize_height))
        
        cv2.imshow("Capturing",frame)

        key = chr(cv2.waitKey(1) & 0xff)

        if key == "q":
            break
        elif key == "s":
            record_image(frame, "images")
        t2 = time.time()

        read_times.append(t1 - t0)
        loop_times.append(t2 - t0)
        while len(read_times) > window_size:
            read_times.pop(0)
        while len(loop_times) > window_size:
            loop_times.pop(0)

        if time.time() - prev_log_time > 1.0:
            prev_log_time = time.time()
            print("Read: %0.2f, Loop: %0.2f" % (len(read_times) / sum(read_times), len(loop_times) / sum(loop_times)))

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
