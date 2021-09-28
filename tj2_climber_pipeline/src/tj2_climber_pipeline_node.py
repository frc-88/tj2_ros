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

import ctypes
# a thread gets killed improperly within CvBridge without this causing segfaults
libgcc_s = ctypes.CDLL('libgcc_s.so.1')

from cv_bridge import CvBridge, CvBridgeError


class TJ2ClimberPipeline(object):
    def __init__(self):
        self.node_name = "tj2_climber_pipeline"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.bridge = CvBridge()

        self.min_distance = rospy.get_param("~min_distance", 0.5)
        self.max_distance = rospy.get_param("~max_distance", 1.5)
        
        self.roi_left = rospy.get_param("~roi_left", 0)
        self.roi_top = rospy.get_param("~roi_top", 0)
        self.roi_right = rospy.get_param("~roi_right", 0)
        self.roi_bottom = rospy.get_param("~roi_bottom", 0)

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
        
        self.debug_image_pub = rospy.Publisher("climber_pipeline_debug/image_raw", Image, queue_size=1)
        self.debug_info_pub = rospy.Publisher("climber_pipeline_debug/camera_info", CameraInfo, queue_size=1)
        self.bar_marker_pub = rospy.Publisher("bar_markers", MarkerArray, queue_size=10)
        
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
        debug_image = self.pipeline(cv2_img, self.debug_image_pub.get_num_connections() > 0)
        t1 = time.time()
        rospy.loginfo_throttle(1, "Pipeline rate: %0.3f" % (1.0 / (t1 - t0)))

        # self.publish_climbers(climbers)
        # self.publish_bar_visualization(climbers)
        self.publish_debug_image(debug_image)
    
    def pipeline(self, depth_image, debug=False):
        # constrain depth image to requested range
        threshold, depth_bounded = cv2.threshold(depth_image, self.max_distance_mm, 65535, cv2.THRESH_TOZERO_INV)
        threshold, depth_bounded = cv2.threshold(depth_bounded, self.min_distance_mm, 65535, cv2.THRESH_TOZERO)

        # convert to 0..255 range so OpenCV algorithms can process it
        # normalized = self.normalize_depth(depth_bounded)
        if debug:
            debug_image = self.normalize_depth(depth_bounded)
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
        
        return debug_image

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

    def publish_climbers(self, climbers):
        pass
    
    def publish_bar_visualization(self, climbers):
        rospy.loginfo_throttle(2, "%s potential climbers in view" % len(climbers))
        markers_msg = MarkerArray()

        for index, points in enumerate(climbers):
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
            marker.lifetime = rospy.Duration(0.05)

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


if __name__ == "__main__":
    node = TJ2ClimberPipeline()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
