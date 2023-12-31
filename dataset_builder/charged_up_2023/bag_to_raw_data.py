import os
import cv2
import argparse
import numpy as np
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from tj2_tools.rosbag_to_file.utils import enumerate_bag, Options
from tj2_tools.training.yolo import YoloFrame, YoloObject
from util import get_labels

parser = argparse.ArgumentParser(description="bag_to_raw_data", add_help=True)
parser.add_argument("path", help="path to convert")
parser.add_argument("--pre-label", action="store_true", help="use labels in the bag to label the images")
parser.add_argument("-o", "--output", default="/opt/tj2/tj2_ros/dataset_builder/data/charged_up_2023_raw/game_pieces/", help="output path")
args = parser.parse_args()

# bag_path = "/opt/tj2/bags/diffyjr_alot_2023-01-21-20-42-10.bag"

# bag_path = "/opt/tj2/bags/diffyjr_alot_2023-01-21-20-38-43.bag"
# bag_path = "/opt/tj2/bags/diffyjr_alot_2023-01-21-20-39-34.bag"
# bag_path = "/opt/tj2/bags/diffyjr_alot_2023-01-21-20-43-47.bag"
bag_path = args.path

output_base_path = args.output

output_path = os.path.join(output_base_path, os.path.splitext(os.path.basename(bag_path))[0])
if not os.path.isdir(output_path):
    os.makedirs(output_path)

bridge = CvBridge()
camera_model = None
labels = get_labels()
objects = []

options = Options(bag_path, output_path)
for topic, msg, timestamp in enumerate_bag(options):
    if "/tj2_zed/obj_det/yolo_objects" == topic:
        if not args.pre_label:
            continue
        if camera_model is None:
            continue
        objects = []
        for object in msg.objects:
            top_right_px = object.bounding_box_2d.corners[0].kp
            bottom_left_px = object.bounding_box_2d.corners[2].kp
            objects.append(YoloObject(
                object.label,
                bottom_left_px[0],
                top_right_px[0],
                bottom_left_px[1],
                top_right_px[1],
                camera_model.width,
                camera_model.height
            ))
    elif camera_model is None and topic == "/tj2_zed/rgb/camera_info" and "camerainfo" in str(type(msg)).lower():
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(msg)
    elif "image" in str(type(msg)).lower() and topic.startswith("/tj2_zed/rgb/"):
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        draw_image = np.copy(image)
        base_path = os.path.join(output_path, str(msg.header.stamp.to_nsec()))
        image_path = base_path + ".jpg"
        if args.pre_label:
            if len(objects) == 0:
                continue
            frame_path = base_path + ".txt"
            frame = YoloFrame(frame_path, image_path, labels)
            for obj in objects:
                frame.add_object(obj)
                draw_image = cv2.rectangle(
                    draw_image,
                    (int(obj.bounding_box.x0), int(obj.bounding_box.y0)),
                    (int(obj.bounding_box.x1), int(obj.bounding_box.y1)),
                    (0, 0, 255),
                    1
                )
            frame.write()
        cv2.imwrite(image_path, image)
        cv2.imshow("image", draw_image)
        key = chr(cv2.waitKey(1) & 0xff)
        if key == 'q':
            break
