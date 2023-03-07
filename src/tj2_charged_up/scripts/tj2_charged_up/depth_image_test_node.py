#!/usr/bin/env python3
import copy
import rospy
import numpy as np
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo

from zed_ros_interfaces.msg import ObjectsStamped
from vision_msgs.msg import Detection3DArray, Detection3D
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from tj2_interfaces.msg import Labels
from tj2_tools.training.yolo import YoloFrame, YoloObject


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
        self.labels_sub = rospy.Subscriber("labels", Labels, self.labels_callback, queue_size=1)
        self.detections_sub = rospy.Subscriber(
            "/tj2_zed/obj_det/yolo_objects", ObjectsStamped, self.objects_callback, queue_size=10
        )

        self.debug_image_pub = rospy.Publisher(
            "debug_image", Image, queue_size=10
        )

    def info_callback(self, msg: CameraInfo):
        self.camera_info_sub.unregister()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

    def labels_callback(self, msg: Labels):
        self.class_names = msg.labels

    def depth_callback(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if (
            rospy.Time.now() - self.objects_timestamp
            > self.stale_detection_threshold
        ):
            rospy.logwarn("Detection is stale. Not computing orientations!")
            return

        for obj in self.objects:
            crop = image[obj.bounding_box.y0:obj.bounding_box.y1, obj.bounding_box.x0:obj.bounding_box.x1]

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
