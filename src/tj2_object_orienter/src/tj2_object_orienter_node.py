import os
from typing import List
import rospy
import open3d
import ros_numpy  # apt install ros-noetic-ros-numpy
import numpy as np
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray, BoundingBox3D
from geometry_msgs.msg import Pose, Vector3, Quaternion, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA


def convert_pc_msg_to_np(pc_msg):
    # Fix rosbag issues, see: https://github.com/eric-wieser/ros_numpy/issues/23
    pc_msg.__class__ = PointCloud2
    offset_sorted = {f.offset: f for f in pc_msg.fields}
    pc_msg.fields = [f for (_, f) in sorted(offset_sorted.items())]

    # Conversion from PointCloud2 msg to np array.
    pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
    return pc_np


class TJ2ObjectOrienter:
    def __init__(self) -> None:
        self.node_name = "tj2_object_orienter"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.rotate_up = open3d.geometry.Geometry3D.get_rotation_matrix_from_xyz(
            (np.deg2rad(90.0), np.deg2rad(90.0), np.deg2rad(90.0))
        )
        self.detections = Detection3DArray()
        self.stale_detection_threshold = rospy.Duration(
            rospy.get_param("~stale_detection_threshold", 0.5)
        )
        self.max_detection_distance = rospy.get_param("~max_detection_distance", 1.0)
        self.class_names_path = rospy.get_param("~class_names_path", "")
        self.class_names = self.load_class_names(self.class_names_path)
        self.marker_color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

        self.point_cloud_sub = rospy.Subscriber(
            "points",
            PointCloud2,
            self.point_cloud_callback,
            queue_size=1,
            buff_size=52428800,
        )
        self.detections_sub = rospy.Subscriber(
            "detections", Detection3DArray, self.detections_callback, queue_size=10
        )

        self.oriented_detections_pub = rospy.Publisher(
            "detections/oriented", Detection3DArray, queue_size=10
        )

        self.detection_markers_pub = rospy.Publisher(
            "detections/oriented/markers", MarkerArray, queue_size=10
        )

    def load_class_names(self, path) -> List:
        if not os.path.isfile(path):
            rospy.logwarn(f"Failed to load class names from {path}")
            return []
        with open(path) as file:
            return file.read().splitlines()

    def point_cloud_callback(self, msg: PointCloud2):
        if (
            rospy.Time.now() - self.detections.header.stamp
            > self.stale_detection_threshold
        ):
            return

        points = convert_pc_msg_to_np(msg)
        pcl = open3d.geometry.PointCloud()
        for detection in self.detections.detections:
            center = np.array(
                [
                    detection.bbox.center.position.x,
                    detection.bbox.center.position.y,
                    detection.bbox.center.position.z,
                ],
                dtype=np.float64,
            )
            size = np.array(
                [detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z],
                dtype=np.float64,
            )
            lower_point = center - size
            upper_point = center + size
            pcl.points = open3d.utility.Vector3dVector(
                points[lower_point <= points <= upper_point]
            )
            oriented_box = self.get_oriented_box_from_cropped_cloud(pcl)
            detection.bbox = oriented_box

    def get_oriented_box_from_cropped_cloud(
        self, pcl: open3d.geometry.PointCloud
    ) -> BoundingBox3D:
        o3d_bb = pcl.get_oriented_bounding_box()
        pca_center = o3d_bb.get_center()
        quat = Rotation.from_matrix(o3d_bb.R @ self.rotate_up).as_quat()
        pose = Pose()
        pose.position.x = pca_center[0]
        pose.position.y = pca_center[1]
        pose.position.z = pca_center[2]
        pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        box3d = BoundingBox3D()
        box3d.center = pose
        box3d.size = Vector3(o3d_bb.extent[0], o3d_bb.extent[1], o3d_bb.extent[2])
        return box3d

    def detections_callback(self, msg: Detection3DArray):
        self.detections = msg

    def add_detection_to_marker_array(self, marker_array, detection_3d_msg):
        cube_marker = self.make_marker(detection_3d_msg, self.marker_color)
        text_marker = self.make_marker(detection_3d_msg, self.marker_color)

        label, count = self.get_detection_label(detection_3d_msg)

        cube_marker.type = Marker.CUBE
        cube_marker.ns = "cube_" + cube_marker.ns

        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text_" + cube_marker.ns
        text_marker.text = "%s_%s|%0.1f" % (
            label,
            count,
            detection_3d_msg.results[0].score * 100,
        )
        text_marker.scale.z = min(text_marker.scale.x, text_marker.scale.y)
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0

        marker_array.markers.append(cube_marker)
        marker_array.markers.append(text_marker)

    def make_marker(self, detection_3d_msg, color):
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = detection_3d_msg.results[0].pose.pose
        marker.header = detection_3d_msg.header
        marker.lifetime = rospy.Duration(1.0)
        label, count = self.get_detection_label(detection_3d_msg)
        marker.ns = label
        marker.id = count

        marker.scale.x = detection_3d_msg.bbox.size.x
        marker.scale.y = detection_3d_msg.bbox.size.y
        marker.scale.z = detection_3d_msg.bbox.size.z
        marker.color = color

        return marker

    def publish_detection_markers(self, detections: Detection3DArray):
        markers = MarkerArray()
        for detection in detections.detections:
            self.add_detection_to_marker_array(markers, detection)
        self.detection_markers_pub.publish(markers)

    def get_detection_label(self, detection_msg):
        return self.get_label(detection_msg.results[0].id)

    def get_label(self, obj_id):
        class_index = obj_id & 0xFFFF
        class_count = obj_id >> 16
        if class_index < len(self.class_names):
            label = self.class_names[class_index]
        else:
            label = f"?? ({class_index})"
        return label, class_count
