#!/usr/bin/env python3
import cv2
import math
import time
import rospy
import torch
import tf_conversions
import numpy as np
import tf2_ros

from cv_bridge import CvBridge
from typing import Optional, List, Tuple
from image_geometry import PinholeCameraModel

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from message_filters import TimeSynchronizer, Subscriber

from tj2_interfaces.msg import (
    GameObjectsStamped,
    GameObject,
    UVBoundingBox,
    XYZBoundingBox,
)
from tj2_tools.transforms import transform_pose

from charged_up_perception.timing_frame import TimingFrame
from charged_up_perception.timing_report import TimingReport
from charged_up_perception.detection import (
    Detection2d,
    BoundingBox2d,
    Detection3d,
    BoundingBox3d,
)
from charged_up_perception.parameters import ChargedUpPerceptionParameters
from charged_up_perception.marker_generator import MarkerGenerator
from charged_up_perception.roi_model import Net


class ChargedUpPerceptionNode:
    def __init__(self) -> None:
        self.node_name = "charged_up_perception"
        rospy.init_node(self.node_name)

        self.model_path = rospy.get_param("~model_path", "models/roi_079000.pkl")
        self.enable_debug_image = rospy.get_param("~enable_debug_image", False)
        self.enable_timing_report = rospy.get_param("~enable_timing_report", False)
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.mask_confidence = 0.5
        self.stand_confidence = 0.5
        self.parameters = ChargedUpPerceptionParameters()
        self.classes = self.parameters.classes
        self.color_map = {"cone": (51, 174, 220), "cube": (164, 58, 71)}
        self.colors: List[Tuple[int, int, int]] = [
            self.color_map[label] for label in self.classes
        ]
        self.marker_generator = MarkerGenerator(self.color_map)
        self.report_generator = TimingReport(num_samples=10)

        self.camera_model: Optional[PinholeCameraModel] = None
        self.bridge = CvBridge()
        self.timings: List[TimingFrame] = []
        self.timing_frame = TimingFrame()

        self._load_model()
        self._warmup_model()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.debug_image_pub = rospy.Publisher(
            "charged_up_perception/debug_image", Image, queue_size=1
        )
        self.depth_debug_image_pub = rospy.Publisher(
            "charged_up_perception/depth_debug_image", Image, queue_size=1
        )
        self.markers_pub = rospy.Publisher(
            "charged_up_perception/markers", MarkerArray, queue_size=10
        )
        self.objects_pub = rospy.Publisher(
            "detections", GameObjectsStamped, queue_size=10
        )
        self.nearest_cone_pub = rospy.Publisher(
            "nearest_cone", PoseStamped, queue_size=10
        )

        self.camera_info_sub = rospy.Subscriber(
            "depth/camera_info", CameraInfo, self.info_callback, queue_size=1
        )
        self.image_sub = Subscriber(
            "color/image",
            Image,
            buff_size=720 * 1280 * 3 * 256 + 1000,
        )
        self.depth_sub = Subscriber(
            "depth/image",
            Image,
            buff_size=720 * 1280 * 8 * 256 + 1000,
        )
        self.time_sync = TimeSynchronizer(
            [self.image_sub, self.depth_sub], queue_size=1
        )
        self.time_sync.registerCallback(self.rgbd_callback)

        self.prev_time = rospy.Time.now()

    def _load_model(self) -> None:
        model_start_time = time.time()
        self.device = torch.device("cuda")
        self.model = Net(self.enable_timing_report).to(self.device)
        self.model.load_state_dict(torch.load(self.model_path))
        self.model.eval()
        model_stop_time = time.time()
        rospy.loginfo(
            f"Model took {model_stop_time - model_start_time:0.4f} seconds to load."
        )

    def _warmup_model(self) -> None:
        rospy.loginfo("Warming up net.")
        warmup_start_time = time.time()
        self.infer(np.random.randint(0, 255, size=(720, 1280, 3), dtype=np.uint8))
        warmup_stop_time = time.time()
        rospy.loginfo(
            f"Model took {warmup_stop_time - warmup_start_time:0.4f} seconds to warmup."
        )

    def info_callback(self, msg: CameraInfo) -> None:
        rospy.loginfo("Got camera model")
        self.camera_info_sub.unregister()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

    def rgbd_callback(self, color_msg: Image, depth_msg: Image) -> None:
        self.compute_detections(color_msg, depth_msg)

    def compute_detections(self, color_msg: Image, depth_msg: Image) -> None:
        if self.camera_model is None:
            rospy.logwarn("Camera model is not loaded! Skip computing detections for this image.")
            return
        else:
            rospy.loginfo_once("Camera model is now loaded.")
        self.timing_frame.start = TimingFrame.now()
        color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        debug_image, detections_2d = self.infer(color_image)
        if debug_image is not None:
            self.debug_image_pub.publish(
                self.bridge.cv2_to_imgmsg(
                    debug_image, encoding="bgr8", header=color_msg.header
                )
            )

        depth_image = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough"
        )
        detections_3d = self.detections_2d_to_3d(
            depth_image, self.camera_model, detections_2d
        )
        self.timing_frame.to_3d = TimingFrame.now()

        nearest_index = self.get_nearest_cone(detections_3d)
        if nearest_index != -1:
            nearest_cone = self.detection_to_game_object(
                detections_2d[nearest_index], detections_3d[nearest_index]
            )
            pose_stamped = PoseStamped()
            pose_stamped.pose = nearest_cone.pose
            pose_stamped.header.frame_id = color_msg.header.frame_id
            pose_stamped = transform_pose(self.tf_buffer, pose_stamped, self.base_frame)
            if pose_stamped is not None:
                self.nearest_cone_pub.publish(pose_stamped)

        self.timing_frame.get_nearest = TimingFrame.now()

        objects = self.detections_to_game_objects(
            color_msg.header.frame_id, detections_2d, detections_3d
        )

        self.objects_pub.publish(objects)

        self.markers_pub.publish(
            self.marker_generator.detections_to_markers(
                depth_msg.header.frame_id, detections_3d
            )
        )

        self.timing_frame.stop = TimingFrame.now()

        if self.enable_timing_report:
            report = self.report_generator.timing_report(self.timing_frame)
            report += "\n"
            report += self.report_generator.object_report(detections_3d, self.classes)
            rospy.loginfo(report)

    def infer(self, img: np.ndarray) -> Tuple[Optional[np.ndarray], List[Detection2d]]:
        resized = cv2.resize(
            img,
            (self.parameters.IMG_W, self.parameters.IMG_H),
            interpolation=cv2.INTER_NEAREST,
        )
        data = torch.tensor(resized.transpose(2, 0, 1)).to(self.device).float() / 255
        data = data.reshape((-1,) + data.shape)

        self.timing_frame.resize = TimingFrame.now()
        segments = self.model(data, mode="testing")  # run inference
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
                debug_image[ct : cb + 1, cl : cr + 1][confident_mask] = self.colors[
                    cls_idx
                ]
                cv2.rectangle(
                    debug_image,
                    (cr + 1, ct),
                    (cl, cb + 1),
                    self.colors[cls_idx],
                    thickness=1,
                )
            angle = float(angle.item())

            is_standing = stand > self.stand_confidence
            is_obstructed = not intact
            if is_standing or self.classes[cls_idx] != "cone":
                angle = 0
            detection = Detection2d(
                self.classes[cls_idx],
                index,
                BoundingBox2d(x_right=cr, y_top=ct, x_left=cl, y_bottom=cb),
                angle,
                is_standing,
                is_obstructed,
                confident_mask,
            )
            detections.append(detection)
            if debug_image is not None:
                debug_image = self.draw_detection(debug_image, detection)
        self.timing_frame.detect2d_prep = TimingFrame.now()
        return debug_image, detections

    def detections_to_game_objects(
        self,
        frame_id: str,
        detections_2d: List[Detection2d],
        detections_3d: List[Detection3d],
    ) -> GameObjectsStamped:
        objects = GameObjectsStamped()
        objects.width = self.parameters.IMG_W
        objects.height = self.parameters.IMG_H
        objects.header.frame_id = frame_id
        assert objects.objects is not None
        for det2d, det3d in zip(detections_2d, detections_3d):
            objects.objects.append(self.detection_to_game_object(det2d, det3d))
        return objects

    def detection_to_game_object(
        self, detection_2d: Detection2d, detection_3d: Detection3d
    ) -> GameObject:
        obj = GameObject()
        obj.label = detection_3d.label
        obj.object_index = detection_3d.index
        obj.class_index = self.parameters.classes.index(detection_3d.label)
        obj.confidence = float(np.average(detection_2d.mask))

        pose = Pose()
        pose.position.x = float(detection_3d.bounding_box.x)
        pose.position.y = float(detection_3d.bounding_box.y)
        pose.position.z = float(detection_3d.bounding_box.z)
        pose.orientation = detection_3d.orientation
        obj.pose = pose

        obj.bounding_box_3d = self.detection_bbox_3d_to_object_bbox_3d(
            detection_3d.bounding_box
        )
        obj.bounding_box_2d = self.detection_bbox_2d_to_object_bbox_2d(
            detection_2d.bounding_box
        )

        return obj

    def detection_bbox_2d_to_object_bbox_2d(self, bbox: BoundingBox2d) -> UVBoundingBox:
        obj_bbox = UVBoundingBox()
        obj_bbox.width = int(abs(bbox.x_left - bbox.x_right))
        obj_bbox.height = int(abs(bbox.y_bottom - bbox.y_top))

        assert obj_bbox.points is not None
        obj_bbox.points[0].x = int(bbox.x_left)
        obj_bbox.points[0].y = int(bbox.y_top)

        obj_bbox.points[1].x = int(bbox.x_right)
        obj_bbox.points[1].y = int(bbox.y_top)

        obj_bbox.points[2].x = int(bbox.x_right)
        obj_bbox.points[2].y = int(bbox.y_bottom)

        obj_bbox.points[3].x = int(bbox.x_left)
        obj_bbox.points[3].y = int(bbox.y_bottom)

        return obj_bbox

    def detection_bbox_3d_to_object_bbox_3d(
        self, bbox: BoundingBox3d
    ) -> XYZBoundingBox:
        obj_bbox = XYZBoundingBox()

        obj_bbox.dimensions.x = bbox.width
        obj_bbox.dimensions.y = bbox.height
        obj_bbox.dimensions.z = bbox.depth

        # TODO: compute bounding box corners

        return obj_bbox

    def draw_detection(self, image: np.ndarray, detection: Detection2d):
        object_name = f"{detection.label}-{detection.index}"
        angle_annotation = (
            f"{int(detection.angle_degrees)}deg"
            if not detection.is_standing
            else "standing"
        )
        obstructed_annotation = "obstructed" if detection.is_obstructed else ""
        named = " ".join([object_name, angle_annotation, obstructed_annotation])
        image = cv2.putText(
            image,
            named,
            (detection.bounding_box.x_left, detection.bounding_box.y_top),
            cv2.FONT_HERSHEY_COMPLEX,
            0.5,
            (255, 255, 0),
            1,
        )
        return image

    def scale_bounding_box(
        self,
        bounding_box: BoundingBox2d,
        source_shape: Tuple[float, float],
        dest_shape: Tuple[float, float],
    ) -> BoundingBox2d:
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

    def get_z_min_max(
        self, detection_2d: Detection2d, depth_image: np.ndarray
    ) -> Tuple[float, float]:
        mask = np.zeros(depth_image.shape[0:2], np.uint8)
        bb = detection_2d.bounding_box
        mask[bb.y_top : bb.y_bottom + 1, bb.x_left : bb.x_right + 1][
            detection_2d.mask
        ] = 1
        depth_masked = depth_image[np.where(mask)]
        if len(depth_masked) == 0:
            rospy.logwarn("Depth mask is empty!")
            return 0.0, 0.0
        z_dist = np.nanmean(depth_masked)
        z_std = np.nanstd(depth_masked)
        z_min = z_dist - z_std * 0.5
        z_max = z_dist + z_std * 0.5
        return z_min, z_max

    def detections_2d_to_3d(
        self,
        depth_image: np.ndarray,
        camera_model: PinholeCameraModel,
        detections_2d: List[Detection2d],
    ) -> List[Detection3d]:
        detections_3d = []
        depth_image_resize = cv2.resize(
            depth_image,
            (self.parameters.IMG_W, self.parameters.IMG_H),
            interpolation=cv2.INTER_NEAREST,
        )
        for detection_2d in detections_2d:
            scaled_box = self.scale_bounding_box(
                detection_2d.bounding_box, depth_image_resize.shape, depth_image.shape
            )
            z_min, z_max = self.get_z_min_max(detection_2d, depth_image_resize)

            top_right_ray = camera_model.projectPixelTo3dRay(
                (scaled_box.x_right, scaled_box.y_top)
            )
            top_right_x_dist = top_right_ray[0] * z_max
            top_right_y_dist = top_right_ray[1] * z_max
            bottom_left_ray = camera_model.projectPixelTo3dRay(
                (scaled_box.x_left, scaled_box.y_bottom)
            )
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

            quat = tf_conversions.transformations.quaternion_from_euler(
                0.0, base_angle, standing_angle
            )
            bounding_box_3d = BoundingBox3d(
                x_dist, y_dist, z_dist, width, height, depth
            )

            orientation = Quaternion()
            orientation.x = quat[0]
            orientation.y = quat[1]
            orientation.z = quat[2]
            orientation.w = quat[3]

            detection_3d = Detection3d(
                detection_2d.label, detection_2d.index, orientation, bounding_box_3d
            )
            detections_3d.append(detection_3d)
        return detections_3d

    def get_nearest_cone(self, detections_3d: List[Detection3d]) -> int:
        nearest_index = -1
        nearest_distance = np.inf
        for index, detection in enumerate(detections_3d):
            if detection.label != "cone":
                continue
            distance = np.linalg.norm(
                np.array(
                    [
                        detection.bounding_box.x,
                        detection.bounding_box.y,
                        detection.bounding_box.z,
                    ]
                )
            )
            if distance < nearest_distance:
                nearest_index = index
        return nearest_index

    def run(self):
        rospy.spin()


def main():
    node = ChargedUpPerceptionNode()
    node.run()


if __name__ == "__main__":
    main()
