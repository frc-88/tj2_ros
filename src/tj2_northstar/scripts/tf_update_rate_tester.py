import tf2_ros
import numpy as np
import time
import rospy
from tj2_tools.transforms import lookup_transform


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


rospy.init_node("listener")

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

try:
    while not rospy.is_shutdown():
        transform = lookup_transform(tf_buffer, "map", "camera_optical_0")
        if transform and transform != prev_transform:
            print(
                f"x={transform.transform.translation.x}, "
                f"y={transform.transform.translation.y}"
            )
            prev_transform = transform
            timestamps.append((transform.header.stamp.to_sec(), time.monotonic()))
finally:
    parse_timestamps()
