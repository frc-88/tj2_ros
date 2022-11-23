#!/usr/bin/env python3
import os
import cv2
import sys
import yaml
import math
import signal
import threading
import numpy as np

import rospy
import rosnode
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

from flask import Response
from flask import Flask
from flask import render_template
from flask import send_from_directory

from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import Image

from vision_msgs.msg import Detection3DArray

from tj2_waypoints.msg import WaypointArray

from tj2_tools.occupancy_grid import OccupancyGridManager
from tj2_tools.robot_state import Pose2d


class WebappNode:
    def __init__(self) -> None:
        rate = rospy.get_param("~rate", 10.0)
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")
        self.footprint_param_path = rospy.get_param("~footprint_param", "/move_base/local_costmap/footprint")
        self.footprint_topic = rospy.get_param("~footprint_topic", "/move_base/local_costmap/footprint")
        self.param_footprint = rospy.get_param(self.footprint_param_path, None)
        self.class_names_path = rospy.get_param("~class_names_path", "")

        self.robot_arrow_length_meters = rospy.get_param("~robot_arrow_length_meters", 1.0)
        self.waypoint_arrow_length_meters = rospy.get_param("~waypoint_arrow_length_meters", 0.5)
        self.detection_radius_meters = rospy.get_param("~detection_radius_meters", 0.3)
        self.map_scale = rospy.get_param("~map_scale", 1.0)
        self.post_scale = rospy.get_param("~post_scale", 1.0)
        self.enable_camera = rospy.get_param("~enable_camera", False)

        self.ogm = OccupancyGridManager()
    
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.robot_pose = Pose2d()
        self.nearest_waypoint_info = ""
        self.nearest_detection_info = ""
        self.waypoints = {}
        
        self.camera_msg = Image()
        self.bridge = CvBridge()

        self.footprint = PolygonStamped()
        if self.param_footprint is not None:
            rospy.loginfo("Loading footprint from parameter")
            if type(self.param_footprint) == str:
                self.param_footprint = yaml.safe_load(self.param_footprint)
            msg = PolygonStamped()
            for point in self.param_footprint:
                pt = Point()
                pt.x = point[0]
                pt.y = point[1]
                msg.polygon.points.append(pt)
            self.footprint_callback(msg)
        else:
            rospy.loginfo("Loading footprint from topic")
            self.footprint_sub = rospy.Subscriber(self.footprint_topic, PolygonStamped, self.footprint_callback, queue_size=1)
            
        if self.class_names_path:
            rospy.loginfo(f"Loading class names from {self.class_names_path}")
            with open(self.class_names_path) as file:
                self.class_names = file.read().splitlines()
        else:
            rospy.loginfo(f"No class names path defined. Not loading class names")
            self.class_names = []
                
        self.detections_msg = Detection3DArray()
        
        self.rate = rospy.Rate(rate)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
        if self.enable_camera:
            self.camera_sub = rospy.Subscriber("/tj2_zed/rgb/image_rect_color", Image, self.camera_callback, queue_size=1)
        self.waypoints_sub = rospy.Subscriber("/tj2/waypoints", WaypointArray, self.waypoints_callback, queue_size=5)
        self.detections_sub = rospy.Subscriber("/tj2_zed/obj_det/detections", Detection3DArray, self.detections_callback, queue_size=5)

    def map_callback(self, msg):
        global lock
        with lock:
            self.ogm = OccupancyGridManager.from_msg(msg)
            self.ogm.set_scale(self.map_scale)

    def footprint_callback(self, msg):
        global lock
        with lock:
            self.footprint = msg

    def camera_callback(self, msg):
        global lock
        with lock:
            self.camera_msg = msg

    def waypoints_callback(self, msg):
        global lock
        with lock:
            for waypoint in msg.waypoints:
                name = waypoint.name
                x = waypoint.pose.position.x
                y = waypoint.pose.position.y
                yaw = euler_from_quaternion((
                    waypoint.pose.orientation.x,
                    waypoint.pose.orientation.y,
                    waypoint.pose.orientation.z,
                    waypoint.pose.orientation.w,
                ))[2]
                self.waypoints[name] = Pose2d(x, y, yaw)

    def detections_callback(self, msg):
        self.detections_msg = msg

    def get_point_in_global_frame(self, transform, x, y):
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = self.robot_frame
        robot_pose.pose.position.x = x
        robot_pose.pose.position.y = y
        robot_pose.pose.orientation.w = 1.0
        global_pose = tf2_geometry_msgs.do_transform_pose(robot_pose, transform)
        return global_pose.pose.position.x, global_pose.pose.position.y

    def get_label(self, obj_id: int):
        index = obj_id & 0xffff
        if index < len(self.class_names):
            return self.class_names[index]
        else:
            return f"?? ({index})"

    def render_robot(self, map_image):
        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            transform = TransformStamped()
            transform.header.frame_id = self.global_frame
            transform.child_frame_id = self.robot_frame
            transform.transform.rotation.w = 1.0

        points = []
        for point in self.footprint.polygon.points:
            robot_x, robot_y = self.get_point_in_global_frame(transform, point.x, point.y)
            img_x, img_y = self.ogm.get_costmap_x_y(robot_x, robot_y)
            points.append([img_x, img_y])
        
        self.robot_pose.x = transform.transform.translation.x
        self.robot_pose.y = transform.transform.translation.y
        self.robot_pose.theta = euler_from_quaternion((
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
        ))[2]
        robot_center = self.ogm.get_costmap_x_y(*self.get_point_in_global_frame(transform, 0.0, 0.0))
        robot_forward = self.ogm.get_costmap_x_y(*self.get_point_in_global_frame(transform, self.robot_arrow_length_meters, 0.0))
        
        if len(points) > 0:
            points = np.array(points, dtype=np.int32)
            points = points.reshape((-1, 1, 2))
            map_image = cv2.polylines(map_image, [points], True, (0, 0, 255), 1)
            map_image = cv2.arrowedLine(map_image, robot_center, robot_forward, (50, 0, 255), 2, tipLength=0.5)
        else:
            dot = self.ogm.get_costmap_x_y(0.0, 0.0)
            map_image = cv2.circle(map_image, dot, 5, (0, 0, 255), -1)
        return map_image

    def render_camera(self, map_image):
        if self.camera_msg.header.stamp == rospy.Time(0):
            return map_image
        camera_image = self.bridge.imgmsg_to_cv2(self.camera_msg, desired_encoding="bgr8")
        camera_width = camera_image.shape[1]
        camera_height = camera_image.shape[0]
        # map_width = map_image.shape[1]
        map_height = map_image.shape[0]
        if camera_height > map_height:
            ratio = map_height / camera_height
            new_width = int(camera_width * ratio)
            camera_image = cv2.resize(camera_image, dsize=(new_width, map_height), interpolation=cv2.INTER_NEAREST)
        else:
            camera_image = np.pad(camera_image, [(0, map_height - camera_height), (0, 0), (0, 0)], mode='constant')
        return np.hstack((map_image, camera_image))

    def render_waypoints(self, map_image):
        min_distance = None
        closest_waypoint = ""
        for name, pose2d in self.waypoints.items():
            distance = self.robot_pose.distance(pose2d)
            if min_distance is None or distance < min_distance:
                min_distance = distance
                closest_waypoint = name
        self.nearest_waypoint_info = f"{closest_waypoint}: {min_distance:0.3f}"
        for name, pose2d in self.waypoints.items():
            point1 = self.ogm.get_costmap_x_y(pose2d.x, pose2d.y)
            point2 = self.ogm.get_costmap_x_y(
                pose2d.x + self.waypoint_arrow_length_meters * math.cos(pose2d.theta),
                pose2d.y + self.waypoint_arrow_length_meters * math.sin(pose2d.theta)
            )
            if name == closest_waypoint:
                color = (255, 0, 10)
                thickness = 2
            else:
                color = (200, 0, 0)
                thickness = 1
            map_image = cv2.arrowedLine(map_image, point1, point2, color, thickness, tipLength=0.5)

        return map_image

    def render_detections(self, map_image):
        if len(self.detections_msg.header.frame_id) == 0 or len(self.detections_msg.detections) == 0:
            self.nearest_detection_info = f"No detections available"
            return map_image
        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame, self.detections_msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            transform = TransformStamped()
            transform.header.frame_id = self.global_frame
            transform.child_frame_id = self.detections_msg.header.frame_id
            transform.transform.rotation.w = 1.0

        min_info = {
            "dist": None,
            "point": (0, 0),
            "radius": 0,
            "name": "No object"
        }
        radius = 0
        
        for detection in self.detections_msg.detections:
            radius = int(
                (max(detection.bbox.size.x, detection.bbox.size.y) / 2.0 + self.detection_radius_meters) /
                self.ogm.resolution
            )
            x = detection.bbox.center.position.x
            y = detection.bbox.center.position.y
            distance = math.sqrt(x * x + y * y)
            robot_x, robot_y = self.get_point_in_global_frame(transform, x, y)
            point = self.ogm.get_costmap_x_y(robot_x, robot_y)
            if radius > 2:
                map_image = cv2.circle(map_image, point, 2, (20, 100, 0), -1)
            map_image = cv2.circle(map_image, point, radius, (20, 100, 0), 1)
            
            if min_info["dist"] is None or distance < min_info["dist"]:
                min_info["dist"] = distance
                min_info["point"] = point
                min_info["radius"] = radius
                min_info["name"] = self.get_label(detection.results[0].id)
        
        if min_info["dist"] is not None:
            if radius > 2:
                map_image = cv2.circle(map_image, min_info["point"], 2, (0, 255, 0), -1)
            map_image = cv2.circle(map_image, min_info["point"], min_info["radius"], (0, 255, 0), 1)
        self.nearest_detection_info = f"{min_info['name']}: {min_info['dist']:0.3f}"

        return map_image

    def render_nearest_points_of_interest(self, map_image):
        map_height = map_image.shape[0]
        map_image = np.pad(map_image, [(0, 50), (0, 0), (0, 0)], mode='constant')
        
        map_image = cv2.putText(map_image, self.nearest_waypoint_info, (10, map_height + 17), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (255, 255, 255), 1)
        map_image = cv2.putText(map_image, self.nearest_detection_info, (10, map_height + 38), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.0, (255, 255, 255), 1)
        return map_image

    def render(self):
        # map renders
        map_image = self.ogm.get_image()
        map_image = self.render_waypoints(map_image)
        map_image = self.render_robot(map_image)
        map_image = self.render_detections(map_image)

        map_image = np.flipud(map_image).astype(np.uint8)

        # post flip renders
        if self.enable_camera:
            map_image = self.render_camera(map_image)
        map_image = self.render_nearest_points_of_interest(map_image)
        
        if self.post_scale != 1.0:
            new_w = int(map_image.shape[1] * self.post_scale)
            new_h = int(map_image.shape[0] * self.post_scale)
            map_image = cv2.resize(map_image, dsize=(new_w, new_h), interpolation=cv2.INTER_NEAREST)

        return map_image

    def generate(self):
        global lock
        while True:
            with lock:
                image = self.render()
                if image.size == 0:
                    image = np.zeros((300, 300))
                flag, encoded = cv2.imencode(".jpg", image)
            if not flag:
                continue
            self.rate.sleep()
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
                bytearray(encoded) + b'\r\n')


def signal_handler(signal, frame):
    rospy.signal_shutdown("end")
    sys.exit(0)


app = Flask(__name__)
node = None
lock = threading.Lock()
node_name = "tj2_webapp"

@app.route('/')
def index():
    return render_template("index.html")


@app.route("/hud_map")
def hud_map():
    global node
    if node is None:
        return
    return Response(node.generate(),
        mimetype = "multipart/x-mixed-replace; boundary=frame")

@app.route('/favicon.ico')
def favicon():
    return send_from_directory(os.path.join(app.root_path, 'static'),
                               'favicon.ico', mimetype='image/vnd.microsoft.icon')

def run_ros():
    global node
    rospy.init_node(node_name, disable_signals=True)
    node = WebappNode()
    rospy.loginfo("ROS node started")
    rospy.spin()

if all([node_name not in name for name in rosnode.get_node_names()]):
    ros_thread = threading.Thread(target=run_ros)
    ros_thread.start()
    signal.signal(signal.SIGINT, signal_handler)
else:
    rospy.logwarn("Running with multiple workers is not compatible with this server!")

if __name__ == '__main__':
    ip_address = rospy.get_param("~host", "0.0.0.0")
    port = rospy.get_param("~port", 5802)
    app.run(ip_address, port, use_reloader=False, threaded=False, debug=False)

