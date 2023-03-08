#!/usr/bin/env python3
import cv2
import math
import rospy
import threading
import numpy as np
from scipy import odr
from typing import List, Tuple, Set
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point
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
        self.lock = threading.Lock()

    def draw_image(self, image: np.ndarray) -> None:
        with self.lock:
            self.image = image

    def draw_houghlines(
        self,
        lines: List,
        color: Tuple[float, float, float] = (0, 255, 255),
    ) -> None:
        with self.lock:
            if self.image is None:
                return
            for line in lines:
                cv2.line(self.image, (line[0], line[1]), line[2], line[3], color)

    def pause(self):
        key = cv2.waitKey(1)
        with self.lock:
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


# type check workaround
cross = lambda x, y: np.cross(x,y)

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
        self.lines = []
        self.objects_timestamp = rospy.Time.now()
        self.camera_model = None
        self.stale_detection_threshold = rospy.Duration(
            rospy.get_param("~stale_detection_threshold", 0.5)
        )
        self.max_detection_distance = rospy.get_param("~max_detection_distance", 1.0)
        self.pair_angle_threshold = 10.0
        self.grounded_frame = "base_link"
        self.transform_tolerance = rospy.Duration(1.0)
        self.bridge = CvBridge()
        self.linear_model = odr.Model(self.linear_func)

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

        self.markers_pub = rospy.Publisher(
            "debug_markers", MarkerArray, queue_size=10
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
        edges = cv2.Canny(image, 50, 200, None, 3)
        self.plotter.draw_image(edges)
        self.lines = []
        for bb in self.get_crops(msg.header.stamp):
            crop = edges[bb.y0:bb.y1, bb.x0:bb.x1]
            rho_thetas = cv2.HoughLines(crop, 1, np.pi / 180, 150, None, 0, 0)
            if rho_thetas is None:
                continue
            lines = [[self.rho_theta_to_xy(line[0][0], line[0][1], image.shape)] for line in rho_thetas]
            lines = HoughBundler.bundle(lines)
            self.plotter.draw_houghlines(lines, (1.0, 0.0, 0.0))
            self.lines.append(lines)

    def depth_callback(self, msg: Image):
        if len(self.lines) == 0:
            rospy.loginfo("No lines found in color image. Can't process depth.")
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        markers = MarkerArray()
        for index, lines in enumerate(self.lines):
            pairs = self.get_pairs(lines, self.pair_angle_threshold)
            result = self.find_cone_lines(pairs, lines)
            if result is None:
                continue
            cone_line_1, cone_line_2 = result
            avg_line = (
                (cone_line_1[0] + cone_line_2[0]) / 2.0,
                (cone_line_1[1] + cone_line_2[1]) / 2.0
                (cone_line_1[2] + cone_line_2[2]) / 2.0
                (cone_line_1[3] + cone_line_2[3]) / 2.0
            )

            pt0, pt1 = self.get_xyz_samples(image, avg_line)
            pose = self.points_to_pose(pt0, pt1)
            line_marker: Marker = self.make_marker("line", index, msg.header.frame_id, pose, (0.0, 1.0, 0.0, 1.0))
            line_marker.pose.orientation.w = 1.0
            line_marker.points.append(Point(*pt0.tolist()))
            line_marker.points.append(Point(*pt1.tolist()))
            line_marker.scale.x = 0.1

            pose_marker: Marker = self.make_marker("pose", index, msg.header.frame_id, pose, (0.0, 1.0, 0.0, 1.0))
            pose_marker.pose = pose
            pose_marker.scale.x = 1.0
            pose_marker.scale.y = 0.05
            pose_marker.scale.z = 0.05

            markers.markers.append(pose_marker)
            markers.markers.append(line_marker)
        self.markers_pub.publish(markers)

    def make_marker(self, ns, index, frame_id, color) -> Marker:
        marker = Marker()
        marker.action = Marker.ADD
        marker.header.frame_id = frame_id
        marker.lifetime = rospy.Duration(10.0)
        marker.ns = ns
        marker.id = index
        marker.color = color
        return marker

    def get_xyz_samples(self, image, avg_line):
        x0, y0, x1, y1 = avg_line
        slope = (x1 - x0) / (y1 - y0)
        intercept = x0 - slope * avg_line[1]
        xs = np.arange(x0, x1)
        ys = self.linear_func((slope, intercept), xs)
        zs = image[xs, ys]
        depth_slope, depth_intercept = self.fit_line(xs, zs)
        z0 = self.linear_func((depth_slope, depth_intercept), x0)
        z1 = self.linear_func((depth_slope, depth_intercept), x1)
        pt0 = np.array([x0, y0, z0])
        pt1 = np.array([x1, y1, z1])
        return pt0, pt1

    def points_to_pose(self, pt0, pt1):
        length_0 = np.linalg.norm(pt0)
        length_1 = np.linalg.norm(pt1)
        x, y, z = cross(pt0, pt1)
        w = np.sqrt((length_0 ** 2) * (length_1 ** 2)) + np.dot(pt0, pt1)
        quaternion = np.array([x, y, z, w])
        norm = np.linalg.norm(quaternion)
        quaternion /= norm
        return self.pose_from_array(np.mean([pt0, pt1]), quaternion)

    def pose_from_array(self, point, quaternion):
        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def get_pairs(self, houghlines: List, pair_angle_threshold: float) -> Set:
        """
        For each hough line, find a corresponding angle with a similar angle.
        If a line doesn't have a pair, it won't appear in the return value
        houghlines format:
        [
            [[x0_a, y0_a, x1_a, y1_a]],
            [[x0_b, y0_b, x1_b, y1_b]],
            ...
        ]

        return set of tuples. Each tuple is length 2. Both values are row indices
            of houghlines.
        """
        angles = []
        for line in houghlines:
            angles.append(self.bundler.get_orientation(line[0]))
        pairs = set()
        for index, angle in enumerate(angles):
            for comp_index, compare_angle in enumerate(angles):
                if index == comp_index:
                    continue
                if abs(angle - compare_angle) < pair_angle_threshold:
                    if index < comp_index:
                        index_pair = (index, comp_index)
                    else:
                        index_pair = (comp_index, index)
                    pairs.add(index_pair)
        return pairs
    
    def find_cone_lines(self, pairs: Set, lines: List):
        longest_pair = None
        longest_distance = 0.0
        if len(pairs) == 0:
            rospy.warn('No pairs found. Failed to find cone.')
            return None
        for pair in pairs:
            length1 = HoughBundler.line_magnitude(lines[0][pair[0]])
            length2 = HoughBundler.line_magnitude(lines[0][pair[1]])
            longer_distance = (length1 + length2) / 2.0
            if longest_pair is None:
                longest_pair = pair
                longest_distance = longer_distance
            if longer_distance > longest_distance:
                longest_pair = pair
                longest_distance = longer_distance
        if longest_pair is None:
            rospy.warn('No pairs found. Failed to find cone.')
            return None
        
        return (lines[0][longest_pair[0]], lines[0][longest_pair[1]])

    def fit_line(
        self, xs: np.ndarray, ys: np.ndarray
    ) -> Tuple[float, float, int, float, float]:
        data = odr.Data(xs, ys)
        myodr = odr.ODR(data, self.linear_model, beta0=[1000.0, 0.0])
        output = myodr.run()
        y_intercept = output.beta[1]
        slope = output.beta[0]
        return slope, y_intercept

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

    @staticmethod
    def linear_func(p: Tuple[float, float], x: float) -> float:
        m, c = p
        return m * x + c

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
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.plotter.pause()


def main():
    node = DepthImageTestNode()
    node.run()


if __name__ == "__main__":
    main()
