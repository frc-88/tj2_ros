#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import uvc
from uvc.uvc_bindings import CameraMode


devices = uvc.device_list()
print("Available devices:")
for index, device in enumerate(devices):
    print(f"{index}: {device}")


captures = []

for index in (1, 2):
    device = devices[index]
    capture = uvc.Capture(device["uid"])
    # print(capture.available_modes)
    mode = CameraMode(1600, 1200, 15, 7, "MJPEG", True)
    capture.frame_mode = mode

    controls = {
        "Auto Exposure Mode": 1,
        "Absolute Exposure Time": 50,
        "Gain": 20,
    }

    for control in capture.controls:
        if control.display_name in controls:
            control.value = controls[control.display_name]
    captures.append(capture)

rospy.init_node("pyuvc_test", anonymous=True)

image_pubs = (
    rospy.Publisher("image1", Image, queue_size=1),
    rospy.Publisher("image2", Image, queue_size=1),
)
bridge = CvBridge()

while not rospy.is_shutdown():
    for capture, image_pub in zip(captures, image_pubs):
        frame = capture.get_frame(timeout=1.0)
        is_bgr = hasattr(frame, "bgr")
        data = frame.bgr if is_bgr else frame.gray
        encoding = "bgr8" if is_bgr else "mono8"
        if frame.data_fully_received:
            image_pub.publish(
                bridge.cv2_to_imgmsg(
                    data, encoding=encoding, header=Header(stamp=rospy.Time.now())
                )
            )
for capture in captures:
    capture.close()
