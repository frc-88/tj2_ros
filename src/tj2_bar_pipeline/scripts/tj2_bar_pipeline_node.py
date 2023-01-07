#!/usr/bin/env python3
import rospy

import cv2
import math
import time
import datetime
import numpy as np

import tf2_ros
import tf_conversions
import tf2_geometry_msgs

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from image_geometry import PinholeCameraModel

from std_msgs.msg import ColorRGBA

from tj2_interfaces.msg import NTEntry

from tj2_tools.robot_state import Pose2d

import ctypes
# a thread gets killed improperly within CvBridge without this causing segfaults
libgcc_s = ctypes.CDLL('libgcc_s.so.1')

from cv_bridge import CvBridge, CvBridgeError


class TJ2BarPipeline(object):
    def __init__(self):
        self.node_name = "tj2_bar_pipeline"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.bridge = CvBridge()

        self.min_distance = rospy.get_param("~min_distance", 0.5)
        self.max_distance = rospy.get_param("~max_distance", 5.0)
        self.contour_perimeter_threshold = rospy.get_param("~contour_perimeter_threshold", 200)
        self.line_angle_lower_threshold = rospy.get_param("~line_angle_lower_threshold", math.pi / 4)
        self.line_angle_upper_threshold = rospy.get_param("~line_angle_upper_threshold", 3 * math.pi / 4)
        self.hough_lines_rho = rospy.get_param("~hough_lines_rho", 1.1)
        self.hough_lines_theta = rospy.get_param("~hough_lines_theta", math.pi / 360.0)
        self.hough_lines_threshold = rospy.get_param("~hough_lines_threshold", 100)
        self.hough_lines_min_length = rospy.get_param("~hough_lines_min_length", 100)
        self.hough_lines_max_gap = rospy.get_param("~hough_lines_max_gap", 100)
        self.bar_z_lower_threshold = rospy.get_param("~bar_z_lower_threshold", 0.5)
        self.bar_z_upper_threshold = rospy.get_param("~bar_z_upper_threshold", 5.0)
        self.z_outlier_stddev = rospy.get_param("~z_outlier_stddev", 3.0)
        self.line_mask_width = rospy.get_param("~line_mask_width", 1)

        self.roi_left = rospy.get_param("~roi_left", 0)
        self.roi_top = rospy.get_param("~roi_top", 20)
        self.roi_right = rospy.get_param("~roi_right", 0)
        self.roi_bottom = rospy.get_param("~roi_bottom", 20)

        self.depth_topic = rospy.get_param("~depth_topic", "depth/image_raw")
        self.info_topic = rospy.get_param("~info_topic", "depth/camera_info")
        self.base_link_frame = rospy.get_param("~base_link_frame", "base_link")

        self.min_distance_mm = int(self.min_distance * 1000.0)
        self.max_distance_mm = int(self.max_distance * 1000.0)

        self.camera_model = None
        self.camera_frame = None
        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.dilate_kernel = np.ones((5, 5), np.uint8)
        
        self.debug_image_pub = rospy.Publisher("bar_pipeline_debug/image_raw", Image, queue_size=1)
        self.debug_info_pub = rospy.Publisher("bar_pipeline_debug/camera_info", CameraInfo, queue_size=1)
        self.bar_marker_pub = rospy.Publisher("bar_markers", MarkerArray, queue_size=10)
        self.nt_pub = rospy.Publisher("nt_passthrough", NTEntry, queue_size=10)

        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.Subscriber(self.info_topic, CameraInfo, self.info_callback, queue_size=1)

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        rospy.spin()
    
    def info_callback(self, msg):
        if self.camera_model is not None:
            return
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

        self.camera_frame = msg.header.frame_id
        self.camera_info = msg

    def depth_callback(self, msg):
        if self.camera_model is None:
            return
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        t0 = time.time()
        bars, debug_image = self.pipeline(cv2_img, self.debug_image_pub.get_num_connections() > 0)
        t1 = time.time()
        rospy.loginfo_throttle(1.0, "Pipeline rate: %0.3f" % (1.0 / (t1 - t0)))

        self.publish_bars(bars)
        self.publish_bar_visualization(bars)
        self.publish_debug_image(debug_image)
    
    def pipeline(self, depth_image, debug=False):
        # constrain depth image to requested range
        threshold, depth_bounded = cv2.threshold(depth_image, self.max_distance_mm, 65535, cv2.THRESH_TOZERO_INV)
        threshold, depth_bounded = cv2.threshold(depth_bounded, self.min_distance_mm, 65535, cv2.THRESH_TOZERO)

        # convert to 0..255 range so OpenCV algorithms can process it
        # normalized = cv2.normalize(depth_bounded, None, 0, 255, cv2.NORM_MINMAX)
        normalized = self.normalize_depth(depth_bounded)
        if debug:
            debug_image = self.normalize_depth(depth_image)
            # debug_image = cv2.applyColorMap(debug_image, cv2.COLORMAP_JET)
            debug_image = cv2.applyColorMap(debug_image, cv2.COLORMAP_OCEAN)
            # debug_image = cv2.cvtColor(debug_image, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(
                debug_image,
                (self.roi_left, self.roi_top),
                (depth_image.shape[1] - self.roi_right, depth_image.shape[0] - self.roi_bottom),
                (0, 0, 255), 2
            )
        else:
            debug_image = None
        
        # remove noise from image
        normalized = cv2.medianBlur(normalized, 3)

        normalized = cv2.dilate(normalized, self.dilate_kernel, iterations=1)

        # find contours and generate an image from them
        contours_image, contours = self.contours(normalized, debug_image)

        # identify lines in the image
        lines, hough_debug_image = self.houghlines(contours_image, debug_image)

        lines = self.filter_lines(normalized, lines)

        # print("lines: ", lines)
        # print('hough image shape: ', hough_debug_image.shape)

        lines, clouds, debug_image = self.hough_line_clouds(depth_bounded, lines, hough_debug_image, debug_image)
        bars = self.bars_from_cloud(lines, clouds)

        return bars, debug_image

    def filter_lines(self, normalized, lines):
        th = 0.8
        h, w = normalized.shape[:2]
        slopes = []
        mid_x = normalized.shape[1] // 2
        for line in lines:
            x1, y1, x2, y2 = line
            k = (y2 - y1) / (x2 - x1)
            b = y1 - k * x1
            mid_y = k * mid_x + b
            if mid_y > 0 and mid_y < normalized.shape[0]:
                slopes.append([mid_y, k, x1, x2, line])
        slopes = sorted(slopes, key=lambda s: s[0])
        if len(slopes) < 2:
            return []
        filtered_lines = []
        for i in range(1, len(slopes)):
            t_slope, b_slope = slopes[i - 1], slopes[i]
            k1, k2 = t_slope[1], b_slope[1]
            mid_y1, mid_y2 = t_slope[0], b_slope[0]
            if mid_y2 - mid_y1 < 4:
                continue
            if abs(k1 - k2) > 0.05:
                continue
            k = (k1 + k2) / 2
            mid_y = (mid_y1 + mid_y2) / 2
            b = mid_y - k * mid_x
            left = min(t_slope[2], b_slope[2])
            right = max(t_slope[3], b_slope[3])
            xs = np.array(list(range(left, right)))
            ys = np.clip(k * xs + b + 0.5, 0, h - 1).astype(np.int32)
            xs = (xs + 0.5).astype(np.int32)
            grays = normalized[ys, xs]
            effects = grays > 0
            ratio = sum(effects) / len(effects)
            # print('r: ', ratio)
            if ratio > th:
                # normalized[ys, xs] = 255
                ly, ry = int(k * left + b), int(k * right + b)
                filtered_line = (left, ly, right, ry)
                filtered_lines.append(filtered_line)
        return filtered_lines

    def normalize_depth(self, depth_image):
        return np.uint8(depth_image.astype(np.float64) * 255 / self.max_distance_mm)

    def publish_debug_image(self, debug_image):
        if debug_image is None:
            return
        try:
            image_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            image_msg.header.frame_id = self.camera_frame
        except CvBridgeError as e:
            rospy.logerr(e)
        else:
            rospy.loginfo_throttle(2, "Publishing debug image")
            self.debug_image_pub.publish(image_msg)
            self.debug_info_pub.publish(self.camera_info)

    def publish_bars(self, bars):
        bar_count = 0
        bar_distance = 0.0
        bar_angle = 0.0
        
        valid_bars = self.get_valid_bars(bars)
        bar_count = len(valid_bars)
    
        min_dist = None
        min_index = -1
        try:
            for index, points in enumerate(valid_bars):
                x0 = points[0][0]
                y0 = points[0][1]
                x1 = points[1][0]
                y1 = points[1][0]
                m = (y1 - y0) / (x1 - x0)
                # calculation from: https://math.stackexchange.com/questions/1796400/estimate-line-in-theta-rho-space-given-2-points
                rho = abs(m * x1 - y1) / math.sqrt(m * m + 1)
                if min_dist is None or rho < min_dist:
                    min_dist = rho
                    min_index = index
        except ValueError:
            print("value error:", valid_bars)

        if min_index >= 0:
            points = bars[min_index]

            pt1 = points[0]
            pt2 = points[1]
            x0 = pt1[0]
            y0 = pt1[1]
            x1 = pt2[0]
            y1 = pt2[1]

            m = (y1 - y0) / (x1 - x0)
            bar_angle = math.atan(m)

            bar_distance = min_dist

        self.nt_pub.publish(NTEntry("bar/distance", bar_distance))
        self.nt_pub.publish(NTEntry("bar/angle", bar_angle))
        self.nt_pub.publish(NTEntry("bar/count", bar_count))
        self.nt_pub.publish(NTEntry("bar/update", rospy.Time.now().to_sec()))
    
    def get_valid_bars(self, bars):
        valid_bars = []
        for points in bars:
            is_valid = True
            for point in points:
                z_height = point[2]
                if not (self.bar_z_lower_threshold <= z_height <= self.bar_z_upper_threshold):
                    is_valid = False
                    break
            if is_valid:
                valid_bars.append(points)
        return valid_bars
    
    def publish_bar_visualization(self, bars):
        rospy.loginfo_throttle(2, "%s potential bars in view" % len(bars))
        markers_msg = MarkerArray()

        for index, points in enumerate(bars):
            marker = Marker()
            marker.action = Marker.ADD
            marker.type = Marker.LINE_STRIP
            marker.ns = "bar"
            marker.id = index
            marker.header.frame_id = self.base_link_frame
            # marker.header.frame_id = self.camera_frame
            marker.pose = Pose()
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.025  # line width
            marker.color = ColorRGBA(0.3, 0.3, 1.0, 1.0)
            # marker.lifetime = rospy.Duration(0.15)
            marker.lifetime = rospy.Duration(0.15)

            for point in points:
                msg_point = Point()
                msg_point.x = point[0]
                msg_point.y = point[1]
                msg_point.z = point[2]
                marker.points.append(msg_point)
                if not (self.bar_z_lower_threshold <= msg_point.z <= self.bar_z_upper_threshold):
                    marker.color = ColorRGBA(1.0, 0.3, 0.3, 1.0)
            
            markers_msg.markers.append(marker)

        self.bar_marker_pub.publish(markers_msg)

    def bars_from_cloud(self, lines, clouds):
        bars = []  # an array of potential bars in the robot's frame
        assert len(lines) == len(clouds)
        for index in range(len(lines)):
            cloud = clouds[index]
            if len(cloud) == 0:
                continue
            line = lines[index]
            bar_coordinates = []
            for point in self.best_fit_3d(line, cloud):
                if point is None:
                    break
                camera_z = point[2] / 1000.0  # Z values in the point cloud are in mm
                result = self.pixels_to_camera_frame(point[0], point[1], camera_z)
                if result is None:
                    rospy.logwarn("Unable to translate pixel coordinates to camera coordinates. No camera model has been received.")
                    break
                camera_x, camera_y = result
                # print("camera: \t%0.4f\t%0.4f\t%0.4f" % (camera_x, camera_y, camera_z))

                result = self.camera_to_robot_frame(camera_x, camera_y, camera_z, timeout=rospy.Duration(0.02))
                if result is None:
                    rospy.logwarn("Unable to translate camera coordinates to robot frame. TF from camera to robot has been received.")
                    break
                robot_x, robot_y, robot_z = result

                # if not (self.bar_z_lower_threshold <= robot_z <= self.bar_z_upper_threshold):
                #     break

                bar_coordinates.append((robot_x, robot_y, robot_z))
                # bar_coordinates.append((camera_x, camera_y, camera_z))
            if len(bar_coordinates) == 2:
                bars.append(bar_coordinates)
        return bars

    def best_fit_3d(self, line, cloud):
        # given a 3d cloud of points, return two 3d points that describes the best
        # fit line.

        x1, y1, x2, y2 = line

        xy_fit = np.polyfit(cloud[:, 0], cloud[:, 1], 1)
        xz_fit = np.polyfit(cloud[:, 0], cloud[:, 2], 1)

        y1_fit = xy_fit[0] * x1 + xy_fit[1]
        y2_fit = xy_fit[0] * x2 + xy_fit[1]
        z1_fit = xz_fit[0] * x1 + xz_fit[1]
        z2_fit = xz_fit[0] * x2 + xz_fit[1]

        return (x1, y1_fit, z1_fit), (x2, y2_fit, z2_fit)
    
    def hough_line_clouds(self, depth_image, lines, hough_debug_image, debug_image):
        # Using lines generated by houghlines, mask pixels that lay on the line.
        # For each column in the image, find the average value (pixel values outside the acceptable range).
        # Each value in the resulting array is a z distance along the discovered line.
        # Each z value is paired with an x, y coordinate
        # a 2D array describing the point cloud is returned
        nonzero_mask = depth_image > 0
        nonzero_mask = np.uint8(nonzero_mask)
        
        clouds = []
        filtered_lines = []
        
        total_mask = np.zeros_like(nonzero_mask)

        for line in lines:
            x1, y1, x2, y2 = line
            mask_image = np.zeros_like(nonzero_mask)
            cv2.line(mask_image, (x1, y1), (x2, y2), (255, 255, 255), self.line_mask_width)
            target_mask = cv2.bitwise_and(mask_image, nonzero_mask)
            if debug_image is not None:
                total_mask = cv2.bitwise_or(total_mask, target_mask)
            
            masked_depth = cv2.bitwise_and(depth_image, depth_image, mask=target_mask)

            column_sums = masked_depth.sum(axis=0)
            inrange_count = ((masked_depth > self.min_distance_mm) & (masked_depth < self.max_distance_mm)).sum(axis=0)
            z_values = np.true_divide(column_sums, inrange_count, where=inrange_count != 0)
            
            cloud = []
            for xn in range(x1, x2):
                zn = z_values[xn]
                if not (self.min_distance_mm <= zn < self.max_distance_mm):
                    continue
                yn = (y2 - y1) / (x2 - x1) * (xn - x1) + y1
                cloud.append((xn, yn, zn))
            if len(cloud) == 0:
                continue

            cloud = np.array(cloud)

            if self.z_outlier_stddev is not None:
                z_cloud = cloud[:, 2]
                z_std = z_cloud.std()
                z_dist = np.abs(z_cloud - z_cloud.mean())
                not_outlier = z_dist < self.z_outlier_stddev * z_std

                cloud = cloud[not_outlier]
            if len(cloud) == 0:
                continue
            clouds.append(cloud)
            filtered_lines.append(line)

        if debug_image is not None:
            masked_debug = cv2.cvtColor(total_mask * 255, cv2.COLOR_GRAY2BGR)
            masked_debug[:, :, 0] = 0
            masked_debug[:, :, 1] = 0
            debug_image = cv2.addWeighted(debug_image, 1.0, masked_debug, 1.0, 0.0)

            # masked_debug = cv2.bitwise_and(debug_image, debug_image, mask=total_mask)
            # debug_image = cv2.vconcat((masked_debug, hough_debug_image))
            
        return filtered_lines, clouds, debug_image
        
    def pixels_to_camera_frame(self, pixel_x, pixel_y, camera_z):
        if self.camera_model is None:
            return None
        ray = self.camera_model.projectPixelTo3dRay((pixel_x, pixel_y))
        camera_x = ray[0] * camera_z
        camera_y = ray[1] * camera_z
        return camera_x, camera_y
    
    def camera_to_robot_frame(self, camera_x, camera_y, camera_z, time_window=None, timeout=None):
        if time_window is None:
            time_window = rospy.Time(0)
        else:
            time_window = rospy.Time.now() - time_window

        if timeout is None:
            timeout = rospy.Duration(1.0)

        try:
            transform = self.tf_buffer.lookup_transform(self.base_link_frame, self.camera_frame, time_window, timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (self.camera_frame, self.base_link_frame, e))
            return None

        camera_pose = PoseStamped()
        camera_pose.header.frame_id = self.camera_frame
        camera_pose.pose.position.x = camera_x
        camera_pose.pose.position.y = camera_y
        camera_pose.pose.position.z = camera_z
        base_link_pose = tf2_geometry_msgs.do_transform_pose(camera_pose, transform)
        
        robot_x = base_link_pose.pose.position.x
        robot_y = base_link_pose.pose.position.y
        robot_z = base_link_pose.pose.position.z

        return robot_x, robot_y, robot_z

    def grab_contours(self, cnts):
        # if the length the contours tuple returned by cv2.findContours
        # is '2' then we are using either OpenCV v2.4, v4-beta, or
        # v4-official
        if len(cnts) == 2:
            cnts = cnts[0]

        # if the length of the contours tuple is '3' then we are using
        # either OpenCV v3, v4-pre, or v4-alpha
        elif len(cnts) == 3:
            cnts = cnts[1]

        # otherwise OpenCV has changed their cv2.findContours return
        # signature yet again and I have no idea WTH is going on
        else:
            raise Exception(("Contours tuple must have length 2 or 3, "
                            "otherwise OpenCV changed their cv2.findContours return "
                            "signature yet again. Refer to OpenCV's documentation "
                            "in that case"))

        # return the actual contours array
        return cnts

    def contours(self, image, debug_image):
        # image = cv2.Canny(image, 50, 150, apertureSize=3)
        contours = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_image = np.zeros_like(image)
        contours = self.grab_contours(contours)
        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            if perimeter < self.contour_perimeter_threshold:
                continue
            cv2.drawContours(contour_image, [contour], -1, (255, 255, 255), 1)
            cv2.drawContours(debug_image, [contour], -1, (255, 0, 255), 1)
        
        return contour_image, contours


    def houghlines(self, edges, debug_image):
        if debug_image is not None:
            debug_image = np.copy(debug_image)
        lines = cv2.HoughLinesP(
            edges,
            self.hough_lines_rho,
            self.hough_lines_theta,
            self.hough_lines_threshold,
            minLineLength=self.hough_lines_min_length,
            maxLineGap=self.hough_lines_max_gap
        )
        result = []
        if lines is not None:
            for hough_line in lines:
                line = hough_line[0]
                x1, y1, x2, y2 = line
                angle = math.atan2(y2 - y1, x2 - x1)
                if not (self.line_angle_lower_threshold < angle < self.line_angle_upper_threshold or
                        -self.line_angle_upper_threshold < angle < -self.line_angle_lower_threshold):
                    line = None

                if x1 < self.roi_left and x2 < self.roi_left:
                    line = None
                if x1 > edges.shape[1] - self.roi_right and x2 > edges.shape[1] - self.roi_right:
                    line = None
                if y1 < self.roi_top and y2 < self.roi_top:
                    line = None
                if y1 > edges.shape[0] - self.roi_bottom and y2 > edges.shape[0] - self.roi_bottom:
                    line = None
                # y1 = edges.shape[0] - y1
                # y2 = edges.shape[0] - y2
                if line is not None:
                    result.append((x1, y1, x2, y2))
                if debug_image is not None:
                    if line is not None:
                        cv2.line(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    else:
                        cv2.line(debug_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        return result, debug_image


if __name__ == "__main__":
    node = TJ2BarPipeline()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
