#!/usr/bin/env python
import math
import time
import copy

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import torch

import rospy
import tf_conversions

from std_msgs.msg import ColorRGBA
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Quaternion, Pose

from message_filters import (
    TimeSynchronizer,
    Subscriber,
)

# from shallow_model import Net
from roi_model import Net
# from roi_long_model import Net


@dataclass
class BoundingBox2d:
    x_right: float
    y_top: float
    x_left: float
    y_bottom: float


@dataclass
class BoundingBox3d:
    x: float
    y: float
    z: float
    width: float
    height: float
    depth: float


@dataclass
class Detection2d:
    label: str
    index: int
    bounding_box: BoundingBox2d
    angle_degrees: float
    is_standing: bool
    is_obstructed: bool
    mask: np.ndarray


@dataclass
class Detection3d:
    label: str
    index: int
    orientation: Quaternion
    bounding_box: BoundingBox3d


class TimingFrame:
    def __init__(self) -> None:
        self.start = 0.0
        self.resize = 0.0
        self.inference = 0.0
        self.detect2d_prep = 0.0
        self.to_3d = 0.0
        self.stop = 0.0

    def copy(self):
        return copy.copy(self)
    
    @classmethod
    def now(cls):
        return time.time()


class RoiRunnerNode:
    def __init__(self) -> None:
        self.node_name = "roi_runner"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        # randomly select a certain amount of anchors as negative samples. For these anchors, calculate the iou of this anchor with all
        # objects, if the iou is greater than iou_neg_thresh, ignore that digit. Both yolo part and roi part need more neg samples.

        # self.model_path = 'models/shallow_074000.pkl'  # shallow_model
        self.model_path = 'models/roi_079000.pkl'  # roi_model
        # self.model_path = 'models/long_281000.pkl'  # roi_long_model

        self.enable_debug_image = True
        self.enable_timing_report = True
        self.colors = np.array([[51, 174, 220], [164, 58, 71]], dtype=np.uint8)
        self.model_width = 512
        self.model_height = 256
        self.mask_confidence = 0.5
        self.stand_confidence = 0.5
        self.classes = ['cone', 'cube']
        
        model_start_time = time.time()
        self.device = torch.device('cuda')
        self.model = Net(self.enable_timing_report).to(self.device)
        self.model.load_state_dict(torch.load(self.model_path))
        self.model.eval()
        model_stop_time = time.time()
        rospy.loginfo(f"Model took {model_stop_time - model_start_time:0.4f} seconds to load.")
        
        self.color_map = {self.classes[index]: self.colors[index] for index in range(len(self.classes))}
        self.depth_msg: Optional[Image] = None
        self.camera_model: Optional[PinholeCameraModel] = None
        self.bridge = CvBridge()
        self.timings: List[TimingFrame] = []
        self.timing_frame = TimingFrame()

        rospy.loginfo("Warming up net.")
        warmup_start_time = time.time()
        self.infer(np.random.randint(0, 255, size=(720, 1280, 3), dtype=np.uint8))
        warmup_stop_time = time.time()
        rospy.loginfo(f"Model took {warmup_stop_time - warmup_start_time:0.4f} seconds to warmup.")

        self.debug_image_pub = rospy.Publisher("roi_runner/debug_image", Image, queue_size=1)
        self.depth_debug_image_pub = rospy.Publisher("roi_runner/depth_debug_image", Image, queue_size=1)
        self.camera_info_sub = rospy.Subscriber("/tj2_zed/depth/camera_info", CameraInfo, self.info_callback, queue_size=1)
        # self.image_sub = rospy.Subscriber("/tj2_zed/rgb/image_rect_color", Image, self.image_callback, queue_size=1, buff_size=720 * 1280 * 3 * 256 + 1000)
        # self.depth_sub = rospy.Subscriber("/tj2_zed/depth/depth_registered", Image, self.depth_callback, queue_size=1, buff_size=720 * 1280 * 8 * 256 + 1000)
        self.image_sub = Subscriber("/tj2_zed/rgb/image_rect_color", Image, buff_size=720 * 1280 * 3 * 256 + 1000)
        self.depth_sub = Subscriber("/tj2_zed/depth/depth_registered", Image, buff_size=720 * 1280 * 8 * 256 + 1000)
        self.time_sync = TimeSynchronizer([self.image_sub, self.depth_sub], queue_size=1)
        self.time_sync.registerCallback(self.rgbd_callback)
        self.markers_pub = rospy.Publisher("roi_runner/markers", MarkerArray, queue_size=10)
        
        self.prev_time = rospy.Time.now()

    def info_callback(self, msg: CameraInfo):
        rospy.loginfo("Got camera model")
        self.camera_info_sub.unregister()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

    def rgbd_callback(self, color_msg, depth_msg):
        self.compute_detections(color_msg, depth_msg)

    def image_callback(self, color_msg: Image):
        if self.depth_msg is None or self.camera_model is None:
            return
        if color_msg.header.stamp - self.depth_msg.header.stamp > rospy.Duration(0.5):
            self.depth_msg = None
            return
        self.compute_detections(color_msg, self.depth_msg)

    def compute_detections(self, color_msg, depth_msg):
        self.timing_frame.start = TimingFrame.now()
        color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        debug_image, detections_2d = self.infer(color_image)
        if debug_image is not None:
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8", header=color_msg.header))

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        detections_3d = self.detections_2d_to_3d(depth_image, self.camera_model, detections_2d)
        self.timing_frame.to_3d = TimingFrame.now()
        self.markers_pub.publish(self.detections_to_markers(depth_msg.header.frame_id, detections_3d))

        self.timing_frame.stop = TimingFrame.now()
        self.timings.append(self.timing_frame.copy())
        
        if len(self.timings) >= 10:
            self.timings.pop(0)
        
        if self.enable_timing_report:
            print("Callback FPS: %0.2f" % (1.0 / np.mean(np.diff([frame.start for frame in self.timings]))))
            print("Overall: %0.4fs" % (np.mean([frame.stop - frame.start for frame in self.timings])))
            print("Resize: %0.4fs" % (np.mean([frame.resize - frame.start for frame in self.timings])))
            print("Infer: %0.4fs" % (np.mean([frame.inference - frame.resize for frame in self.timings])))
            print("Detect prep: %0.4fs" % (np.mean([frame.detect2d_prep - frame.inference for frame in self.timings])))
            print("2D to 3D: %0.4fs" % (np.mean([frame.to_3d - frame.detect2d_prep for frame in self.timings])))
            object_counts = {name: 0 for name in self.classes}
            for detection in detections_3d:
                object_counts[detection.label] += 1
            print("Object counts:")
            for name, count in object_counts.items():
                print(f"\t{name}: {count}")
            print()

    def depth_callback(self, msg: Image):
        rospy.loginfo_once("Received depth image")
        self.depth_msg = msg

    def infer(self, img: np.ndarray) -> Tuple[Optional[np.ndarray], List[Detection2d]]:
        resized = cv2.resize(img, (self.model_width, self.model_height), interpolation=cv2.INTER_NEAREST)
        data = torch.tensor(resized.transpose(2,0,1)).to(self.device).float() / 255
        data = data.reshape((-1,)+data.shape)

        self.timing_frame.resize = TimingFrame.now()
        segments = self.model(data, mode='testing')  # run inference
        self.timing_frame.inference = TimingFrame.now()
        
        if self.enable_debug_image:
            debug_image = np.copy(resized)
        else:
            debug_image = None
        detections = []
        for index, segment in enumerate(segments):
            img_idx, cls_idx, ct, cb, cl, cr, intact, stand, angle, sub_mask = segment
            confident_mask = sub_mask > self.mask_confidence

            if debug_image is not None:
                debug_image[ct:cb + 1, cl:cr + 1][confident_mask] = self.colors[cls_idx]
                cv2.rectangle(debug_image, (cr + 1, ct), (cl, cb + 1), self.colors[cls_idx].tolist(), thickness=1)
            angle = float(angle.item())
            
            is_standing = stand > self.stand_confidence 
            is_obstructed = not intact
            if is_standing or self.classes[cls_idx] != 'cone':
                angle = 0
            detection = Detection2d(
                self.classes[cls_idx],
                index,
                BoundingBox2d(
                    x_right=cr, 
                    y_top=ct, 
                    x_left=cl, 
                    y_bottom=cb
                ),
                angle,
                is_standing,
                is_obstructed,
                confident_mask
            )
            detections.append(detection)
            if debug_image is not None:
                debug_image = self.draw_detection(debug_image, detection)
        self.timing_frame.detect2d_prep = TimingFrame.now()
        return debug_image, detections

    def draw_detection(self, image: np.ndarray, detection: Detection2d):
        named = (
            f"{detection.label}-{detection.index} "
            f"{int(detection.angle_degrees)}deg "
            f"{'S' if detection.is_standing else 'D'}{'-O' if detection.is_obstructed else ''}"
        )
        image = cv2.putText(
            image,
            named,
            (detection.bounding_box.x_left, detection.bounding_box.y_top),
            cv2.FONT_HERSHEY_COMPLEX,
            0.5, (255, 255, 0), 1
        )
        return image

    def detections_to_markers(self, frame_id: str, detections: List[Detection3d]) -> MarkerArray:
        markers = MarkerArray()
        for detection in detections:
            pose_marker, box_marker = self.detection_to_marker(frame_id, detection)
            markers.markers.append(pose_marker)
            markers.markers.append(box_marker)
        return markers

    def detection_to_marker(self, frame_id, detection: Detection3d) -> Tuple[Marker, Marker]:
        pose_marker = self.make_base_marker(frame_id, detection)
        box_marker = self.make_base_marker(frame_id, detection)

        pose_marker.type = Marker.ARROW
        pose_marker.ns += "-ARROW"
        pose_marker.scale.x = 1.0
        pose_marker.scale.y = 0.05
        pose_marker.scale.z = 0.05
        
        box_marker.type = Marker.CUBE
        box_marker.ns += "-CUBE"
        box_marker.scale.x = detection.bounding_box.width
        box_marker.scale.y = detection.bounding_box.height
        box_marker.scale.z = detection.bounding_box.depth
        box_marker.color.a = 0.5
        box_marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        
        return pose_marker, box_marker

    def make_base_marker(self, frame_id, detection: Detection3d):
        pose = Pose()
        pose.position.x = float(detection.bounding_box.x)
        pose.position.y = float(detection.bounding_box.y)
        pose.position.z = float(detection.bounding_box.z)
        pose.orientation = detection.orientation

        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = pose
        marker.header.frame_id = frame_id
        marker.lifetime = rospy.Duration(1.0)
        marker.ns = str(detection.label)
        marker.id = int(detection.index)
        color_array = self.color_map[detection.label] / 255.0
        marker.color = ColorRGBA(
            color_array[2],
            color_array[1],
            color_array[0],
            1.0
        )
        return marker

    def scale_bounding_box(self, bounding_box: BoundingBox2d, source_shape, dest_shape) -> BoundingBox2d:
        source_y = source_shape[0]
        source_x = source_shape[1]
        dest_y = dest_shape[0]
        dest_x = dest_shape[1]
        height_scale = dest_y / source_y
        width_scale = dest_x / source_x
        return BoundingBox2d(
            bounding_box.x_right * width_scale,
            bounding_box.y_top * height_scale,
            bounding_box.x_left * width_scale,
            bounding_box.y_bottom * height_scale,
        )

    def get_z_min_max(self, detection_2d: Detection2d, depth_image: np.ndarray) -> Tuple[float, float]:
        mask = np.zeros(depth_image.shape[0:2], np.uint8)
        bb = detection_2d.bounding_box
        mask[bb.y_top:bb.y_bottom+1, bb.x_left:bb.x_right + 1][detection_2d.mask] = 1
        depth_masked = depth_image[np.where(mask)]
        if len(depth_masked) == 0:
            rospy.logwarn('Depth mask is empty!')
            return 0.0, 0.0
        z_dist = np.nanmean(depth_masked)
        z_std = np.nanstd(depth_masked)
        z_min = z_dist - z_std * 0.5
        z_max = z_dist + z_std * 0.5
        return z_min, z_max

    def detections_2d_to_3d(self, depth_image: np.ndarray, camera_model: PinholeCameraModel, detections_2d: List[Detection2d]) -> List[Detection3d]:
        detections_3d = []
        depth_image_resize = cv2.resize(depth_image, (self.model_width, self.model_height), interpolation=cv2.INTER_NEAREST)
        for detection_2d in detections_2d:
            scaled_box = self.scale_bounding_box(detection_2d.bounding_box, depth_image_resize.shape, depth_image.shape)
            z_min, z_max = self.get_z_min_max(detection_2d, depth_image_resize)

            top_right_ray = camera_model.projectPixelTo3dRay((scaled_box.x_right, scaled_box.y_top))
            top_right_x_dist = top_right_ray[0] * z_max
            top_right_y_dist = top_right_ray[1] * z_max
            bottom_left_ray = camera_model.projectPixelTo3dRay((scaled_box.x_left, scaled_box.y_bottom))
            bottom_left_x_dist = bottom_left_ray[0] * z_min
            bottom_left_y_dist = bottom_left_ray[1] * z_min

            x_dist = (top_right_x_dist + bottom_left_x_dist) / 2.0
            y_dist = (top_right_y_dist + bottom_left_y_dist) / 2.0
            z_dist = (z_max + z_min) / 2.0

            width = bottom_left_x_dist - top_right_x_dist
            height = bottom_left_y_dist - top_right_y_dist
            depth = z_max - z_min
            
            if detection_2d.is_standing:
                standing_angle = -math.pi / 2.0
                base_angle = 0.0
            else:
                standing_angle = 0.0
                base_angle = 2 * math.pi - math.radians(detection_2d.angle_degrees)

            quat = tf_conversions.transformations.quaternion_from_euler(0.0, base_angle, standing_angle)
            bounding_box_3d = BoundingBox3d(x_dist, y_dist, z_dist, width, height, depth)

            orientation = Quaternion()
            orientation.x = quat[0]
            orientation.y = quat[1]
            orientation.z = quat[2]
            orientation.w = quat[3]

            detection_3d = Detection3d(detection_2d.label, detection_2d.index, orientation, bounding_box_3d)
            detections_3d.append(detection_3d)
        return detections_3d

    def run(self):
        rospy.spin()


def main():
    node = RoiRunnerNode()
    node.run()


if __name__ == "__main__":
    main()

