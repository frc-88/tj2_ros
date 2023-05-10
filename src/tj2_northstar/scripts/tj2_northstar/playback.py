#!/usr/bin/env python3
import yaml
import rospy
import tqdm
from pynput.keyboard import Listener, Key
from typing import List, Optional
from dataclasses import dataclass
from rosbag import Bag
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from tj2_interfaces.msg import CameraInfoArray
from cv_bridge import CvBridge


@dataclass
class AppData:
    image_pubs: List[rospy.Publisher]
    info_pubs: List[rospy.Publisher]
    bridge: CvBridge


def process_info(data: AppData, msg: CameraInfoArray):
    info_pubs = data.info_pubs
    num_cameras = len(info_pubs)
    assert msg.cameras is not None
    for index in range(num_cameras):
        info_pubs[index].publish(msg.cameras[index])


def process_image(data: AppData, msg: Image):
    bridge = data.bridge
    image_pubs = data.image_pubs
    num_cameras = len(image_pubs)

    image = bridge.imgmsg_to_cv2(msg, "passthrough")

    height, width = image.shape[0:2]
    subwidth = width // num_cameras
    for index in range(num_cameras):
        x0 = index * subwidth
        x1 = (index + 1) * subwidth
        sub_image = image[0:height, x0:x1]
        header = Header()
        header.frame_id = f"camera_optical_{index}"
        header.stamp = msg.header.stamp
        sub_msg = bridge.cv2_to_imgmsg(sub_image, msg.encoding, header)
        image_pubs[index].publish(sub_msg)


def load_camera_info(path: str) -> Optional[CameraInfo]:
    with open(path) as file:
        config = yaml.safe_load(file)
    info = CameraInfo()
    info.height = config.get("height")
    info.width = config.get("width")
    info.distortion_model = config.get("distortion_model", "plumb_bob")
    info.D = config.get("D")
    info.K = config.get("K")
    info.R = config.get("R")
    info.P = config.get("P")
    info.binning_x = config.get("binning_x", 0)
    info.binning_y = config.get("binning_y", 0)
    roi = config.get("roi", None)
    if roi is not None:
        info.roi = RegionOfInterest(
            x_offset=roi.get("x_offset", 0),
            y_offset=roi.get("y_offset", 0),
            height=roi.get("height", 0),
            width=roi.get("width", 0),
            do_rectify=roi.get("do_rectify", False),
        )
    return info


def main():
    rospy.init_node("arducam_playback")

    camera_info_directory = str(rospy.get_param("~info_directory", "."))

    bag_path = "/home/tj2/tj2_ros/bags/arducam_2023-05-10-14-32-51.bag"
    bag = Bag(bag_path)

    NUM_CAMERAS = 4
    image_pubs = []
    for index in range(NUM_CAMERAS):
        image_pubs.append(
            rospy.Publisher(f"/northstar/camera_{index}/image_raw", Image, queue_size=1)
        )
    combined_image_pub = rospy.Publisher(
        "/northstar/camera/image_raw", Image, queue_size=1
    )
    info_pubs = []
    info_msgs = []
    for index in range(NUM_CAMERAS):
        info_pubs.append(
            rospy.Publisher(
                f"/northstar/camera_{index}/camera_info", CameraInfo, queue_size=1
            )
        )
        info_msg = load_camera_info(camera_info_directory + f"/camera_{index}.yaml")
        info_msg.header.frame_id = f"camera_optical_{index}"
        info_msgs.append(info_msg)
    combined_info_msg = CameraInfoArray()
    combined_info_msg.cameras = info_msgs

    combined_info_pub = rospy.Publisher(
        "/northstar/camera/camera_info", CameraInfoArray, queue_size=1
    )
    bridge = CvBridge()
    paused = False
    data = AppData(image_pubs, info_pubs, bridge)

    def on_press(key):
        nonlocal paused
        if key == Key.space:
            paused = not paused
            if paused:
                print("Pausing")
            else:
                print("Unpausing")

    listener = Listener(on_press=on_press)
    listener.start()

    try:
        messages = bag.read_messages()
        length = bag.get_message_count()
        pbar = tqdm.tqdm(total=length)

        while True:
            start_real_time = rospy.Time.now()
            start_bag_time = None
            for topic, msg, timestamp in messages:
                if start_bag_time is None:
                    start_bag_time = timestamp
                real_time = rospy.Time.now()
                relative_real_duration = real_time - start_real_time
                relative_bag_duration = timestamp - start_bag_time

                if relative_bag_duration > relative_real_duration:
                    rospy.sleep(
                        (relative_bag_duration - relative_real_duration).to_sec()
                    )

                while paused:
                    if rospy.is_shutdown():
                        break
                    rospy.sleep(0.25)
                pbar.update(1)
                if rospy.is_shutdown():
                    break

                for index in range(NUM_CAMERAS):
                    info_msgs[index].header.stamp = timestamp

                if topic == "/northstar/camera/image_raw":
                    process_image(data, msg)
                    combined_image_pub.publish(msg)
                    combined_info_pub.publish(combined_info_msg)
                    for index in range(NUM_CAMERAS):
                        info_pubs[index].publish(info_msgs[index])

            messages = bag.read_messages()
            if rospy.is_shutdown():
                break
            pbar.update(-length)

    finally:
        bag.close()


if __name__ == "__main__":
    main()
