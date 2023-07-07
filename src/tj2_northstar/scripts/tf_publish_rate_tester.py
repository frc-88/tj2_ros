import numpy as np
import time
import rospy
from tf2_msgs.msg import TFMessage


timestamps = []


def parse_timestamps():
    data = np.array(timestamps)
    headers = data[:, 0]
    monotonic = data[:, 1]

    headers_rate = 1.0 / np.mean(np.diff(headers))
    monotonic_rate = 1.0 / np.mean(np.diff(monotonic))

    print(f"{headers_rate=}")
    print(f"{monotonic_rate=}")


prev_transform = None


def callback(msg):
    global prev_transform
    for transform_msg in msg.transforms:
        parent = transform_msg.header.frame_id
        if parent == "map":
            print(
                transform_msg.transform.translation.x,
                transform_msg.transform.translation.y,
            )
            if transform_msg.transform != prev_transform:
                prev_transform = transform_msg.transform
                timestamps.append(
                    (transform_msg.header.stamp.to_sec(), time.monotonic())
                )


rospy.init_node("listener")

rospy.Subscriber("/tf", TFMessage, callback)

try:
    rospy.spin()
finally:
    parse_timestamps()
