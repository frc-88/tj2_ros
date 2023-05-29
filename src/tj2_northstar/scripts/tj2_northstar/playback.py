#!/usr/bin/env python3
import yaml
import time
import rospy
import tqdm
from pynput.keyboard import Listener, Key
from typing import List, Optional
from dataclasses import dataclass
from rosbag import Bag
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from tj2_interfaces.msg import CameraInfoArray
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros
from rosgraph_msgs.msg import Clock


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


def process_image(data: AppData, msg: Image, timestamp: Optional[rospy.Time] = None):
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
        if timestamp is None:
            header.stamp = msg.header.stamp
        else:
            header.stamp = timestamp
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
    bag_name = str(rospy.get_param("~bag_name", ""))

    bag_path = f"/opt/tj2/tj2_ros/bags/{bag_name}.bag"
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
    odom_pub = rospy.Publisher("/tj2/odom", Odometry, queue_size=10)
    # clock_pub = rospy.Publisher("clock", Clock, queue_size=10)

    tf_broadcaster = tf2_ros.TransformBroadcaster()
    bridge = CvBridge()
    paused = False
    pressed_keys = {}
    data = AppData(image_pubs, info_pubs, bridge)

    sim_clock = Clock()

    def on_press(key):
        pressed_keys[key] = True
        nonlocal paused
        if pressed_keys.get(Key.space, False) and pressed_keys.get(Key.ctrl, False):
            paused = not paused
            if paused:
                print("Pausing")
            else:
                print("Unpausing")

    def on_release(key):
        pressed_keys[key] = False

    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        messages = bag.read_messages()
        length = bag.get_message_count()
        pbar = tqdm.tqdm(total=length)

        while True:
            start_real_time = time.monotonic()
            start_bag_time = None
            for topic, msg, timestamp in messages:
                if start_bag_time is None:
                    start_bag_time = timestamp
                real_time = time.monotonic()
                relative_real_duration = real_time - start_real_time
                relative_bag_duration = (timestamp - start_bag_time).to_sec()
                now = rospy.Time.now()

                if relative_bag_duration > relative_real_duration:
                    time.sleep(relative_bag_duration - relative_real_duration)
                sim_clock.clock = timestamp

                while paused:
                    if rospy.is_shutdown():
                        break
                    time.sleep(0.25)
                pbar.update(1)
                if rospy.is_shutdown():
                    break

                for index in range(NUM_CAMERAS):
                    info_msgs[index].header.stamp = now

                if topic == "/northstar/camera/image_raw":
                    process_image(data, msg, now)
                    combined_image_pub.publish(msg)
                    combined_info_pub.publish(combined_info_msg)
                    for index in range(NUM_CAMERAS):
                        info_pubs[index].publish(info_msgs[index])
                elif topic == "/tj2/odom":
                    msg.header.stamp = now
                    odom_pub.publish(msg)
                # elif topic == "/tf":
                #     tf_broadcaster.sendTransform(msg.transforms)
                #     for transform_msg in msg.transforms:
                #         transform_msg.header.stamp = now
                #         parent = transform_msg.header.frame_id
                #         child = transform_msg.child_frame_id
                #         if parent == "odom" and child == "base_link":
                #             tf_broadcaster.sendTransform(transform_msg)
                # clock_pub.publish(sim_clock)

            messages = bag.read_messages()
            if rospy.is_shutdown():
                break
            pbar.update(-length)

    finally:
        bag.close()


if __name__ == "__main__":
    main()
