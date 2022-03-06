import cv2
import json
import time
import numpy as np


def get_info_time(info, index):
    segment = info[index + 1]
    return segment[0]


def get_info_stats(info):
    timestamps = []
    for segment in info[1:]:
        video_time = segment[0]
        timestamps.append(video_time)
    timestamps = np.array(timestamps)
    fps = 1.0 / np.diff(timestamps)
    return np.mean(fps), np.std(fps)


def main():
    window_name = "video"
    # video_path = "/home/ben/tj2_ros/tj2_match_watcher/bags/week0/2022_robot_2022-02-19-13-52-49-video"
    # video_path = "/home/ben/tj2_ros/tj2_match_watcher/bags/week0/2022_robot_2022-02-19-14-20-43-video"
    video_path = "/home/ben/tj2_ros/tj2_match_watcher/bags/week0/2022_robot_2022-02-19-14-57-14-video"
    # video_path = "/home/ben/tj2_ros/tj2_match_watcher/bags/week0/2022_robot_2022-02-19-15-16-59-video"

    cv2.namedWindow(window_name)

    capture_path = video_path + ".mp4"
    info_path = video_path + ".json"

    capture = cv2.VideoCapture(capture_path)
    with open(info_path) as file:
        info = json.load(file)

    real_start_t = time.time()
    video_start_t = get_info_time(info, 0)

    mean_fps, std_fps = get_info_stats(info)
    print("FPS mean=%0.4f, std=%0.4f" % (mean_fps, std_fps))

    frame_num = 0

    def next_frame():
        nonlocal frame_num, image
        success, image = capture.read()
        frame_num += 1
        return success
    
    def reset_start_times():
        nonlocal real_start_t, video_start_t
        real_start_t = time.time()
        video_start_t = get_info_time(info, frame_num)

    while True:
        real_duration = time.time() - real_start_t
        video_duration = get_info_time(info, frame_num) - video_start_t
        if real_duration > video_duration:
            delta = real_duration - video_duration
        else:
            delta = video_duration - real_duration

        print("Sleeping for: %0.4fs" % delta)
        if 0.0 < delta < 1.0:
            time.sleep(delta)
        else:
            reset_start_times()

        if not next_frame():
            break

        image = cv2.putText(image, "%0.3f" % video_duration, (10, 40), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 0), thickness=3)
        cv2.imshow(window_name, image)
        value = cv2.waitKey(1)
        key = chr(value & 0xff)

        if key == 'q':
            break

if __name__ == '__main__':
    main()
