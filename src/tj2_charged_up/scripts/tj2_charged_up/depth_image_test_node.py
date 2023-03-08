#!/usr/bin/env python3
import cv2
import math
import rospy
import threading
import numpy as np
from scipy import odr
from scipy.spatial.transform import Rotation
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
from tj2_tools.training.training_object import BoundingBox
from hough_bundler import HoughBundler


class DebugPlotter:
    def __init__(self) -> None:
        self.window_name = "plot"
        cv2.namedWindow(self.window_name)
        self.image = None
        self.lock = threading.Lock()

    def draw_image(self, image: np.ndarray) -> None:
        with self.lock:
            self.image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    def draw_houghlines(
        self,
        lines: List,
        color: Tuple[float, float, float],
    ) -> None:
        with self.lock:
            if self.image is None:
                return
            for bundle in lines:
                line = bundle[0]
                cv2.line(self.image, (line[0], line[1]), (line[2], line[3]), color, 1)

    def draw_line(
        self,
        line: List,
        color: Tuple[float, float, float],
    ) -> None:
        with self.lock:
            if self.image is None:
                return
            cv2.line(self.image, (line[0], line[1]), (line[2], line[3]), color, 1)

    def draw_box(self, x0, y0, x1, y1, color: Tuple[float, float, float]):
        with self.lock:
            if self.image is None:
                return
            cv2.rectangle(self.image, (x0, y0), (x1, y1), color, 3)

    def pause(self):
        key = cv2.waitKey(1)
        if chr(key & 0xff) == 'q':
            quit()
        if self.image is None:
            return
        with self.lock:
            cv2.imshow(self.window_name, self.image)
            self.image = None


class NoopDebugPlotter:
    def __init__(self) -> None:
        pass

    def draw_image(self, image: np.ndarray) -> None:
        pass

    def draw_houghlines(
        self,
        lines: List,
        color: Tuple[float, float, float],
    ) -> None:
        pass
    
    def draw_line(
        self,
        line: List,
        color: Tuple[float, float, float],
    ) -> None:
        pass
    
    def draw_box(self, x0, y0, x1, y1, color: Tuple[float, float, float]):
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
        self.lines_lock = threading.Lock()
        self.objects_timestamp = rospy.Time.now()
        self.camera_model = None
        self.stale_detection_threshold = rospy.Duration(
            rospy.get_param("~stale_detection_threshold", 0.5)
        )
        self.max_detection_distance = rospy.get_param("~max_detection_distance", 1.0)
        self.cone_label = rospy.get_param("~cone_label", "cone")
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
        # self.color_sub = rospy.Subscriber(
        #     "color",
        #     Image,
        #     self.color_callback,
        #     queue_size=1,
        # )

        self.markers_pub = rospy.Publisher(
            "cone_markers", MarkerArray, queue_size=10
        )
        self.labels_sub = rospy.Subscriber("labels", Labels, self.labels_callback, queue_size=1)
        self.detections_sub = rospy.Subscriber(
            "objects", ObjectsStamped, self.objects_callback, queue_size=10
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
        agg_lines = self.find_detected_lines(msg.header.stamp, image)
        with self.lines_lock:
            self.lines = agg_lines

    def find_detected_lines(self, stamp, image):
        image = cv2.pyrDown(image)
        image = cv2.GaussianBlur(image, (5, 5), 0)
        edges = cv2.Canny(image, 1, 30, None, 3)
        self.plotter.draw_image(edges)
        agg_lines = []
        for bb in self.get_crops(stamp):
            x0 = min(bb.x0, bb.x1) // 2
            x1 = max(bb.x0, bb.x1) // 2
            y0 = min(bb.y0, bb.y1) // 2
            y1 = max(bb.y0, bb.y1) // 2
            self.plotter.draw_box(x0, y0, x1, y1, (255, 0, 0))
            crop = edges[y0:y1, x0:x1]
            lines = cv2.HoughLinesP(crop, 1, np.pi / 180, 10, minLineLength=2, maxLineGap=3)
            if lines is None:
                rospy.loginfo("Hough lines returned nothing")
                continue
            lines = HoughBundler.bundle(lines, 10.0, 10.0)
            lines[..., 0] += x0
            lines[..., 1] += y0
            lines[..., 2] += x0
            lines[..., 3] += y0
            self.plotter.draw_houghlines(lines, (0, 255, 0))
            agg_lines.append(((x0, y0, x1, y1), lines))
        return agg_lines

    def depth_callback(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        image = np.clip(image, 0.0, 2.0)
        norm_image = cv2.normalize(image, None, 0.0, 255.0, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        markers = MarkerArray()
        results = []
        agg_lines = self.find_detected_lines(msg.header.stamp, norm_image)
        with self.lines_lock:
            for index, (bb, lines) in enumerate(agg_lines):
                result = self.find_cone_line(bb, lines)
                self.plotter.draw_line(result, (0, 0, 255))
                results.append(result)

                px_pt0, px_pt1 = self.get_xyz_samples(image, result * 2)
                pt0 = self.pixel_to_point(px_pt0)
                pt1 = self.pixel_to_point(px_pt1)
                
                pose = self.points_to_pose(pt0, pt1)
                line_marker: Marker = self.make_marker("line", index, msg.header.frame_id, (0.0, 1.0, 0.0, 1.0))
                line_marker.pose.orientation.w = 1.0
                line_marker.points.append(Point(*pt0))
                line_marker.points.append(Point(*pt1))
                line_marker.scale.x = 0.01

                pose_marker: Marker = self.make_marker("pose", index, msg.header.frame_id, (0.0, 0.0, 1.0, 1.0))
                pose_marker.pose = pose
                pose_marker.scale.x = 1.0
                pose_marker.scale.y = 0.05
                pose_marker.scale.z = 0.05

                markers.markers.append(pose_marker)
                markers.markers.append(line_marker)
        self.markers_pub.publish(markers)
    
    def pixel_to_point(self, point):
        if self.camera_model is None:
            return None
        ray = self.camera_model.projectPixelTo3dRay((point[0], point[1]))
        x = ray[0] * point[2]
        y = ray[1] * point[2]
        return np.array([x, y, point[2]])

    def make_marker(self, ns, index, frame_id, color) -> Marker:
        marker = Marker()
        marker.action = Marker.ADD
        marker.header.frame_id = frame_id
        marker.lifetime = rospy.Duration(0.5)
        marker.ns = ns
        marker.id = index
        marker.color = ColorRGBA(*color)
        return marker

    def get_xyz_samples(self, image, avg_line):
        x0, y0, x1, y1 = avg_line
        # slope = (y1 - y0) / (x1 - x0)
        # intercept = y0 - slope * x0
        # xs = np.array(np.arange(x0, x1), dtype=int)
        # ys = np.array(self.linear_func((slope, intercept), xs), dtype=int)
        # indices = (0 <= ys) & (ys < image.shape[0]) & (0 <= xs) & (xs < image.shape[1])
        # xs = xs[indices]
        # ys = ys[indices]
        # zs = image[ys, xs]
        # depth_slope, depth_intercept = self.fit_line(xs, zs)
        # z0 = self.linear_func((depth_slope, depth_intercept), x0)
        # z1 = self.linear_func((depth_slope, depth_intercept), x1)
        z0 = image[y0, x0]
        z1 = image[y1, x1]
        pt0 = np.array([x0, y0, z0])
        pt1 = np.array([x1, y1, z1])
        return pt0, pt1

    def points_to_pose(self, pt0, pt1):
        relative_pt = pt1 - pt0
        unit = np.array([1.0, 0.0, 0.0])
        x, y, z = cross(unit, relative_pt)
        length_0 = np.linalg.norm(relative_pt)
        length_1 = np.linalg.norm(unit)
        length = np.sqrt((length_0 ** 2) * (length_1 ** 2))
        w = length + np.dot(relative_pt, unit)
        quaternion = np.array([x, y, z, w])
        norm = np.linalg.norm(quaternion)
        quaternion /= norm
        return self.pose_from_array(np.mean([pt0, pt1], axis=0), quaternion)

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
            angles.append(HoughBundler.get_orientation(line[0]))
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
    
    def find_cone_line(self, bounding_box: Tuple, lines: List, offset_scale=0.1):
        longest_line_index = np.argmax([HoughBundler.line_magnitude(line[0]) for line in lines])
        longest_line = lines[longest_line_index][0]
        height = abs(bounding_box[3] - bounding_box[1])
        longest_line[1] += height * offset_scale
        longest_line[3] += height * offset_scale
        return longest_line
    
    def find_paired_cone_lines(self, pairs: Set, lines: List):
        longest_pair = None
        longest_distance = 0.0
        if len(pairs) == 0:
            rospy.logwarn('No pairs found. Failed to find cone.')
            return None
        for pair in pairs:
            length1 = HoughBundler.line_magnitude(lines[pair[0]][0])
            length2 = HoughBundler.line_magnitude(lines[pair[1]][0])
            longer_distance = (length1 + length2) / 2.0
            if longest_pair is None:
                longest_pair = pair
                longest_distance = longer_distance
            if longer_distance > longest_distance:
                longest_pair = pair
                longest_distance = longer_distance
        if longest_pair is None:
            rospy.logwarn('No pairs found. Failed to find cone.')
            return None
        
        return (lines[longest_pair[0]][0], lines[longest_pair[1]][0])

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
        max_y = image_shape[0]
        max_x = image_shape[1]
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
            if object.label != self.cone_label:
                continue
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
