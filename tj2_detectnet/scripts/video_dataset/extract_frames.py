import os
import cv2
import time
import numpy as np


def process_video(path, output_dir):
    filename = os.path.basename(path)
    name = os.path.splitext(filename)[0]
    output_dir = os.path.join(output_dir, name)
    image_name_format = os.path.join(output_dir, "%s_%%s.png" % name)

    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    print("Loading %s" % path)
    video = cv2.VideoCapture(path)

    blur_threshold = 24.0
    paused = False

    last_motion_frame = None
    frame_num = -1
    saved_frame_num = 0
    motion_frame_width = 300
    frame_skip = 3

    t0 = time.time()
    try:
        while True:
            if not paused:
                success, frame = video.read()
                if not success:
                    break
                frame_num += 1
                if frame_num % frame_skip == 0:
                    continue

                should_save_frame = True
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                height, width = frame.shape[0:2]

                motion_frame_height = int(height * motion_frame_width / width)

                gray = cv2.resize(gray, (motion_frame_width, motion_frame_height))
                gray = cv2.GaussianBlur(gray, (21, 21), 0)
                if last_motion_frame is None:
                    last_motion_frame = gray

                frame_delta = cv2.absdiff(last_motion_frame, gray)
                thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
                thresh = cv2.dilate(thresh, None, iterations=2)

                if np.any(thresh > 128):
                    last_motion_frame = gray
                    motion_text = "Movement"
                else:
                    motion_text = ""
                    should_save_frame = False

                variance = cv2.Laplacian(frame, cv2.CV_64F).var()
                if variance < blur_threshold:
                    should_save_frame = False
                    blur_text = "blurry"
                else:
                    blur_text = "not blurry"

                draw_frame = cv2.resize(np.copy(frame), (motion_frame_width, motion_frame_height))
                draw_frame = cv2.putText(draw_frame, "{}: {:.2f}".format(blur_text, variance), (10, 30),
                                         cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
                cv2.putText(draw_frame, motion_text, (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 3)

                cv2.imshow('frame', draw_frame)
                if should_save_frame:
                    cv2.imwrite(image_name_format % saved_frame_num, frame)
                    saved_frame_num += 1

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord(' '):
                paused = not paused
    finally:
        t1 = time.time()
        video.release()
        cv2.destroyAllWindows()
        print("Took %ss" % (t1 - t0))


def main():
    # filename = "tj2_02-27-2021_1.mkv"
    # filename = "tj2_02-27-2021_2.mkv"
    filenames = ["tj2_02-27-2021_%s.mkv" % x for x in range(3, 13)]

    for filename in filenames:
        path = "/Users/Woz4tetra/Google Drive/Projects/TJ2 ROS/2021 Game Objects Training Data/%s" % filename
        output_dir = "./output"

        process_video(path, output_dir)


if __name__ == '__main__':
    main()
