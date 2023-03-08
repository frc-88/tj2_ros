#!/usr/bin/env python3
import cv2
import time
import rospy
import tf2_ros
import threading
import numpy as np
from scipy import odr
import tf2_geometry_msgs
from typing import List, Tuple
from cv_bridge import CvBridge

from geometry_msgs.msg import Pose, Point, PoseStamped, PoseArray, Quaternion
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from zed_interfaces.msg import ObjectsStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

from tj2_interfaces.msg import Labels
from tj2_tools.training.yolo import YoloObject
from tj2_tools.robot_state.simple_filter import SimpleFilter
from hough_bundler import HoughBundler

# type check workaround
cross = lambda x, y: np.cross(x,y)

class ConeFinder:
    def __init__(self) -> None:
        self.node_name = "cone_finder"
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
        self.max_detection_distance = rospy.get_param("~max_detection_distance", 2.0)
        self.cone_label = rospy.get_param("~cone_label", "cone")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")
        
        self.bundle_min_distance = 10.0
        self.bundle_min_angle = 10.0
        self.hough_rho = 1
        self.hough_theta = np.pi / 180
        self.hough_threshold = 10
        self.hough_min_length = 2
        self.hough_max_gap = 3
        self.gauss_k = 5
        self.canny_lower = 1
        self.canny_upper = 30
        self.canny_aperture_size = 3
        self.bounding_box_pull_down_scale = 0.1
        self.quaternion_smooth_k = 0.5
        
        self.transform_tolerance = rospy.Duration(1.0)
        self.bridge = CvBridge()
        self.linear_model = odr.Model(self.linear_func)
        self.filters = [SimpleFilter(self.quaternion_smooth_k) for _ in range(4)]

        np.set_printoptions(threshold=np.inf)
        
        self.plotter = DebugPlotter()
        # self.plotter = NoopDebugPlotter()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.info_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("depth", Image, self.depth_callback, queue_size=1)
        self.markers_pub = rospy.Publisher("cones/markers", MarkerArray, queue_size=10)
        self.cones_pub = rospy.Publisher("cones/detections", PoseArray, queue_size=10)
        self.nearest_cone_pub = rospy.Publisher("cones/nearest", PoseStamped, queue_size=10)
        self.labels_sub = rospy.Subscriber("labels", Labels, self.labels_callback, queue_size=1)
        self.detections_sub = rospy.Subscriber("objects", ObjectsStamped, self.objects_callback, queue_size=10)

    def info_callback(self, msg: CameraInfo):
        rospy.loginfo("Got camera model")
        self.camera_info_sub.unregister()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

    def labels_callback(self, msg: Labels):
        self.class_names = msg.labels

    def find_detected_lines(self, stamp, image):
        image = cv2.pyrDown(image)
        image = cv2.GaussianBlur(image, (self.gauss_k, self.gauss_k), 0)
        edges = cv2.Canny(image, self.canny_lower, self.canny_upper, None, self.canny_aperture_size)
        self.plotter.draw_image(edges)
        agg_lines = []
        for bb in self.get_crops(stamp):
            x0 = min(bb.x0, bb.x1) // 2
            x1 = max(bb.x0, bb.x1) // 2
            y0 = min(bb.y0, bb.y1) // 2
            y1 = max(bb.y0, bb.y1) // 2
            self.plotter.draw_box(x0, y0, x1, y1, (255, 0, 0))
            crop = edges[y0:y1, x0:x1]
            lines = cv2.HoughLinesP(
                crop,
                self.hough_rho,
                self.hough_theta,
                self.hough_threshold,
                minLineLength=self.hough_min_length,
                maxLineGap=self.hough_max_gap
            )
            if lines is None:
                rospy.loginfo("Hough lines returned nothing")
                continue
            lines = HoughBundler.bundle(lines, self.bundle_min_distance, self.bundle_min_angle)
            lines[..., 0] += x0
            lines[..., 1] += y0
            lines[..., 2] += x0
            lines[..., 3] += y0
            self.plotter.draw_houghlines(lines, (0, 255, 0))
            agg_lines.append(((x0, y0, x1, y1), lines))
        return agg_lines

    def depth_callback(self, msg: Image):
        try:
            robot_tf = self.tf_buffer.lookup_transform(self.robot_frame, msg.header.frame_id, rospy.Time(0), self.transform_tolerance)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (self.robot_frame, msg.header.frame_id, e))
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        image = np.clip(image, 0.0, self.max_detection_distance)
        norm_image = cv2.normalize(image, None, 0.0, 255.0, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        markers = MarkerArray()
        pose_array = PoseArray()
        pose_array.header.frame_id = self.robot_frame
        pose_array.header.stamp = msg.header.stamp
        
        agg_lines = self.find_detected_lines(msg.header.stamp, norm_image)
        nearest_distance = None
        nearest_pose = PoseStamped()
        for index, (bb, lines) in enumerate(agg_lines):
            result = self.find_cone_line(bb, lines)
            self.plotter.draw_line(result, (0, 0, 255))

            px_pt0, px_pt1 = self.get_xyz_samples(image, result * 2)
            if px_pt0 is None or px_pt1 is None:
                continue
            pt0 = self.pixel_to_point(px_pt0)
            pt1 = self.pixel_to_point(px_pt1)
            dist0 = np.linalg.norm(pt0)
            dist1 = np.linalg.norm(pt1)
            nearest_dist = dist0 if dist0 < dist1 else dist1                
            
            pose = self.points_to_pose(pt0, pt1)
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose
            self.add_markers(markers, index, pt0, pt1, pose_stamped)
            robot_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, robot_tf)
            pose_array.poses.append(robot_pose.pose)
            if nearest_distance is None or nearest_dist < nearest_distance:
                nearest_pose = robot_pose
                nearest_distance = nearest_dist

        self.cones_pub.publish(pose_array)
        self.markers_pub.publish(markers)
        if nearest_distance is not None:
            self.update_nearest_pose(nearest_pose)
            self.nearest_cone_pub.publish(nearest_pose)
    
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

    def update_nearest_pose(self, pose: PoseStamped):
        new_quat = [0.0 for _ in range(len(self.filters))]
        for index, value in enumerate((
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )):
            new_quat[index] = self.filters[index].update(value)
        pose.pose.orientation = Quaternion(*new_quat)

    def add_markers(self, markers, index, pt0, pt1, pose_stamped):
        line_marker: Marker = self.make_marker("line", index, pose_stamped.header.frame_id, (0.0, 1.0, 0.0, 1.0))
        line_marker.pose.orientation.w = 1.0
        line_marker.points.append(Point(*pt0))
        line_marker.points.append(Point(*pt1))
        line_marker.scale.x = 0.01

        pose_marker: Marker = self.make_marker("pose", index, pose_stamped.header.frame_id, (0.0, 0.0, 1.0, 1.0))
        pose_marker.pose = pose_stamped.pose
        pose_marker.scale.x = 1.0
        pose_marker.scale.y = 0.05
        pose_marker.scale.z = 0.05

        markers.markers.append(pose_marker)
        markers.markers.append(line_marker)
    
    def pixel_to_point(self, point):
        if self.camera_model is None:
            return None
        ray = self.camera_model.projectPixelTo3dRay((point[0], point[1]))
        x = ray[0] * point[2]
        y = ray[1] * point[2]
        return np.array([x, y, point[2]], dtype=np.float64)

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
        if x1 == x0:
            return None, None
        slope = (y1 - y0) / (x1 - x0)
        intercept = y0 - slope * x0
        xs = np.array(np.arange(x0, x1), dtype=int)
        ys = np.array(self.linear_func((slope, intercept), xs), dtype=int)
        indices = (0 <= ys) & (ys < image.shape[0]) & (0 <= xs) & (xs < image.shape[1])
        xs = xs[indices]
        ys = ys[indices]
        zs = image[ys, xs]
        depth_slope, depth_intercept = self.fit_line(xs, zs)
        z0 = self.linear_func((depth_slope, depth_intercept), x0)
        z1 = self.linear_func((depth_slope, depth_intercept), x1)
        if np.isnan(z0) or np.isnan(z1):
            z0 = image[y0, x0]
            z1 = image[y1, x1]
        if np.isnan(z0) or np.isnan(z1):
            return None, None
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
    
    def find_cone_line(self, bounding_box: Tuple, lines: List):
        longest_line_index = np.argmax([HoughBundler.line_magnitude(line[0]) for line in lines])
        longest_line = lines[longest_line_index][0]
        height = abs(bounding_box[3] - bounding_box[1])
        longest_line[1] += height * self.bounding_box_pull_down_scale
        longest_line[3] += height * self.bounding_box_pull_down_scale
        return longest_line

    def fit_line(
        self, xs: np.ndarray, ys: np.ndarray
    ) -> Tuple[float, float, int, float, float]:
        data = odr.Data(xs, ys)
        myodr = odr.ODR(data, self.linear_model, beta0=[1.0, 1.0])
        output = myodr.run()
        y_intercept = output.beta[1]
        slope = output.beta[0]
        return slope, y_intercept

    @staticmethod
    def linear_func(p: Tuple[float, float], x: float) -> float:
        m, c = p
        return m * x + c

    def get_crops(self, now):
        if (
            now - self.objects_timestamp
            > self.stale_detection_threshold
        ):
            rospy.logwarn("Detection is stale. Not computing orientations!")
            return

        for obj in self.objects:
            yield obj.bounding_box

    def run(self):
        if type(self.plotter) == NoopDebugPlotter:
            rospy.spin()
        else:
            while not rospy.is_shutdown():
                time.sleep(0.1)
                self.plotter.pause()


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


def main():
    node = ConeFinder()
    node.run()


if __name__ == "__main__":
    main()
