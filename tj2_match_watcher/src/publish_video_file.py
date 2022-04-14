#!/usr/bin/python3
import os
import cv2
import json
import time
import numpy as np
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


current_time = rospy.Time(0)

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

def clock_callback(msg):
    global current_time
    current_time = msg.clock

def main():
    window_name = "video"
    rospy.init_node(
        "publish_video_file"
    )
    bag_name = rospy.get_param("~bag_name", "something.bag")

    if bag_name.startswith("/"):
        video_path = bag_name
    else:
        rospack = rospkg.RosPack()
        root_dir = rospack.get_path("tj2_match_watcher")
        # root_dir = "/media/storage"
        base_name = os.path.splitext(bag_name)[0]
        video_path = os.path.join(root_dir, "bags", "video-%s" % base_name)

    bridge = CvBridge()
    image_pub = rospy.Publisher("image", Image, queue_size=10)
    clock_sub = rospy.Subscriber("/clock", Clock, clock_callback)

    cv2.namedWindow(window_name)

    capture_path = video_path + ".mp4"
    info_path = video_path + ".json"

    capture = cv2.VideoCapture(capture_path)
    with open(info_path) as file:
        info = json.load(file)

    while current_time == rospy.Time(0):
        rospy.sleep(0.01)
    real_start_t = current_time.to_sec()
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
        real_start_t = current_time.to_sec()
        video_start_t = get_info_time(info, frame_num)

    while True:
        real_duration = current_time.to_sec() - real_start_t
        video_duration = get_info_time(info, frame_num) - video_start_t
        if real_duration > video_duration:
            delta = real_duration - video_duration
        else:
            delta = video_duration - real_duration

        print("Sleeping for: %0.4fs" % delta)
        if 0.0 < delta < 1.0:
            rospy.sleep(rospy.Duration(delta))
        else:
            reset_start_times()

        if not next_frame():
            break

        if rospy.is_shutdown():
            break

        image = cv2.putText(image, "%0.3f" % video_duration, (10, 40), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 0), thickness=3)
        image_msg = bridge.cv2_to_imgmsg(image, "bgr8")
        image_pub.publish(image_msg)
        # cv2.imshow(window_name, image)
        # value = cv2.waitKey(1)
        # key = chr(value & 0xff)

        # if key == 'q':
        #     break

if __name__ == '__main__':
    main()
