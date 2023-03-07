#!/usr/bin/env python3
import cv2
import math
import rospy
import matplotlib
import numpy as np
from typing import List, Tuple
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo

from zed_interfaces.msg import ObjectsStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from tj2_interfaces.msg import Labels
from tj2_tools.training.yolo import YoloObject
from hough_bundler import HoughBundler


class DebugPlotter:
    def __init__(self) -> None:
        self.window_name = "plot"
        cv2.namedWindow(self.window_name)
        self.image = None

    def draw_image(self, image: np.ndarray) -> None:
        self.image = image

    def draw_houghlines(
        self,
        lines: List,
        color: Tuple[float, float, float] = (0, 255, 255),
    ) -> None:
        if self.image is None:
            return
        for line in lines:
            cv2.line(self.image, (line[0], line[1]), line[2], line[3], color)

    def pause(self):
        key = cv2.waitKey(1)
        cv2.imshow(self.window_name, self.image)
        if chr(key & 0xff) == 'q':
            quit()
        self.image = None


class NoopDebugPlotter:
    def __init__(self) -> None:
        pass

    def draw_image(self, image: np.ndarray) -> None:
        pass

    def draw_houghlines(
        self,
        lines: List,
        color: Tuple[float, float, float] = (0.0, 1.0, 1.0),
    ) -> None:
        pass
    
    def pause(self):
        pass


class DepthImageTestNode:
    def __init__(self) -> None:
        self.node_name = "depth_image_test"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.class_names = []
        self.objects = []
        self.objects_timestamp = rospy.Time.now()
        self.camera_model = None
        self.stale_detection_threshold = rospy.Duration(
            rospy.get_param("~stale_detection_threshold", 0.5)
        )
        self.max_detection_distance = rospy.get_param("~max_detection_distance", 1.0)
        self.grounded_frame = "base_link"
        self.transform_tolerance = rospy.Duration(1.0)
        self.bridge = CvBridge()

        self.marker_color = ColorRGBA(1.0, 0.0, 0.0, 0.2)
        self.arrow_color = ColorRGBA(1.0, 0.0, 1.0, 1.0)
        np.set_printoptions(threshold=np.inf)
        
        self.plotter = DebugPlotter()
        # self.plotter = NoopDebugPlotter()

        self.camera_info_sub = rospy.Subscriber(
            "camera_info",
            CameraInfo,
            self.info_callback,
            queue_size=1,
        )
        self.depth_sub = rospy.Subscriber(
            "depth",
            Image,
            self.depth_callback,
            queue_size=1,
        )
        self.color_sub = rospy.Subscriber(
            "color",
            Image,
            self.color_callback,
            queue_size=1,
        )
        self.labels_sub = rospy.Subscriber("labels", Labels, self.labels_callback, queue_size=1)
        self.detections_sub = rospy.Subscriber(
            "objects", ObjectsStamped, self.objects_callback, queue_size=10
        )

        self.debug_image_pub = rospy.Publisher(
            "debug_image", Image, queue_size=10
        )

    def info_callback(self, msg: CameraInfo):
        rospy.loginfo("Got camera model")
        self.camera_info_sub.unregister()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

    def labels_callback(self, msg: Labels):
        self.class_names = msg.labels

    def color_callback(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.plotter.draw_image(image)
        for bb in self.get_crops(msg.header.stamp):
            crop = image[bb.y0:bb.y1, bb.x0:bb.x1]
            edges = cv2.Canny(crop, 50, 200, None, 3)
            rho_thetas = cv2.HoughLines(edges, 1, np.pi / 180, 150, None, 0, 0)
            if rho_thetas is None:
                continue
            lines = [self.rho_theta_to_xy(line[0][0], line[0][1], image.shape) for line in rho_thetas]
            lines = HoughBundler.bundle(lines)
            self.plotter.draw_houghlines(lines, (1.0, 0.0, 0.0))
        self.plotter.pause()

    def depth_callback(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        for bb in self.get_crops(msg.header.stamp):
            crop = image[bb.y0:bb.y1, bb.x0:bb.x1]

    def rho_theta_to_xy(self, rho, theta, image_shape, length=1000):
        max_y, max_x = image_shape
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + length * -b), int(y0 + length * a))
        pt2 = (int(x0 - length * -b), int(y0 - length * a))
        return [
            self.clamp_dimension(pt1[0], max_x),
            self.clamp_dimension(pt1[1], max_y),
            self.clamp_dimension(pt2[0], max_x),
            self.clamp_dimension(pt2[1], max_y),
        ]

    def clamp_dimension(self, value, max_value):
        return max(0, min(value, max_value))

    def get_crops(self, now):
        if (
            now - self.objects_timestamp
            > self.stale_detection_threshold
        ):
            rospy.logwarn("Detection is stale. Not computing orientations!")
            return

        for obj in self.objects:
            yield obj.bounding_box

    def objects_callback(self, msg: ObjectsStamped):
        if not self.camera_model:
            return
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
                self.camera_model.width,
                self.camera_model.height
            ))
        self.objects = objects
        self.objects_timestamp = msg.header.stamp

    def get_label(self, obj_id):
        class_index = obj_id & 0xFFFF
        class_count = obj_id >> 16
        if class_index < len(self.class_names):
            label = self.class_names[class_index]
        else:
            label = f"?? ({class_index})"
        return label, class_count

    def run(self):
        rospy.spin()


def main():
    node = DepthImageTestNode()
    node.run()


if __name__ == "__main__":
    main()
