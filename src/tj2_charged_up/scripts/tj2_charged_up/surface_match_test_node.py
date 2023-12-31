#!/usr/bin/env python3
import copy
import time
import os
from typing import List, Optional
import rospy
import open3d
import ros_numpy  # apt install ros-noetic-ros-numpy
import numpy as np
from scipy.spatial.transform import Rotation
import cv2

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Vector3, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header


def convert_pc_msg_to_np(pc_msg):
    # Fix rosbag issues, see: https://github.com/eric-wieser/ros_numpy/issues/23
    pc_msg.__class__ = PointCloud2
    offset_sorted = {f.offset: f for f in pc_msg.fields}
    pc_msg.fields = [f for (_, f) in sorted(offset_sorted.items())]

    # Conversion from PointCloud2 msg to np array.
    pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
    return pc_np

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

def convert_pc_o3d_to_ros(open3d_cloud, frame_id):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)


class SurfaceMatchTestNode:
    def __init__(self) -> None:
        self.node_name = "o3d_pca_test"
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
        self.mesh_path = rospy.get_param("~mesh_path", "")
        detector_sampling_step = rospy.get_param("~detector_sampling_step", 0.025)
        detector_distance_step = rospy.get_param("~detector_distance_step", 0.05)
        self.scene_sampling_step = rospy.get_param("~scene_sampling_step", 1.0 / 40.0)
        self.scene_distance_step = rospy.get_param("~scene_distance_step", 0.05)
        detector_num_angles_degrees = rospy.get_param("~detector_num_angles_degrees", 30.0)
        self.class_names = self.load_class_names(self.class_names_path)
        self.marker_color = ColorRGBA(1.0, 0.0, 0.0, 0.5)
        
        self.detector = cv2.ppf_match_3d_PPF3DDetector(
            detector_sampling_step,
            detector_distance_step,
            detector_num_angles_degrees
        )
        
        rospy.loginfo("Loading model")
        model = cv2.ppf_match_3d.loadPLYSimple(self.mesh_path, 1)
        model_o3d = open3d.io.read_point_cloud(self.mesh_path)
        self.model_bb = model_o3d.get_oriented_bounding_box()
        self.model_size = Vector3(self.model_bb.extent[0], self.model_bb.extent[1], self.model_bb.extent[2])
        rospy.loginfo("Model loaded")
        
        rospy.loginfo("Training surface match detector")
        t0 = time.time()
        self.detector.trainModel(model)
        t1 = time.time()
        rospy.loginfo(f"Surface match detector trained in {t1 - t0} seconds")
        
        np.set_printoptions(threshold=np.inf)

        self.point_cloud_sub = rospy.Subscriber(
            "points",
            PointCloud2,
            self.point_cloud_callback,
            queue_size=1,
            buff_size=52428800,
        )
        self.debug_point_cloud_pub = rospy.Publisher(
            "points_debug",
            PointCloud2,
            queue_size=1
        )
        self.detections_sub = rospy.Subscriber(
            "detections", Detection3DArray, self.detections_callback, queue_size=10
        )

        self.oriented_detections_pub = rospy.Publisher(
            "detections/oriented", Detection3DArray, queue_size=10
        )

        self.detection_markers_pub = rospy.Publisher(
            "detections/markers", MarkerArray, queue_size=10
        )

        self.oriented_detection_markers_pub = rospy.Publisher(
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
        oriented_detections = Detection3DArray()
        oriented_detections.header = self.detections.header
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
            size = np.abs(size)
            lower_point = center - size / 2.0
            upper_point = center + size / 2.0
            indices = np.all(np.logical_and(lower_point <= points, points <= upper_point), axis=1)
            if not np.any(indices):
                return
            pcl.points = open3d.utility.Vector3dVector(points[indices])
            self.debug_point_cloud_pub.publish(convert_pc_o3d_to_ros(pcl, msg.header.frame_id))

            oriented_box = self.get_oriented_box_from_cropped_cloud(points[indices].astype(np.float32))
            if oriented_box is None:
                continue
            
            oriented_detection = copy.deepcopy(detection)
            oriented_detection.header = detection.header
            oriented_detection.bbox = oriented_box
            oriented_detection.results[0].pose.pose = oriented_box.center
            oriented_detections.detections.append(oriented_detection)
        self.publish_detection_markers(self.oriented_detection_markers_pub, oriented_detections)

    def get_oriented_box_from_cropped_cloud(
        self, pcl: np.ndarray
    ) -> Optional[BoundingBox3D]:
        print("Matching", pcl.shape)
        results = self.detector.match(pcl, 1.0/40.0, 0.05)
        print(f"Found {len(results)} results")
        time.sleep(0.5)
        if len(results) == 0:
            return None
        
        result = results[0]
        print("Pose to Model Index %d: NumVotes = %d, Residual = %f\n%s\n" % (result.modelIndex, result.numVotes, result.residual, result.pose))
        
        quat = Rotation.from_matrix(result.pose[0:3, 0:3]).as_quat()
        pose = Pose()
        pose.position.x = result.pose[0, 3]
        pose.position.y = result.pose[1, 3]
        pose.position.z = result.pose[2, 3]
        pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        box3d = BoundingBox3D()
        box3d.center = pose
        box3d.size = self.model_size
        return box3d

    def detections_callback(self, msg: Detection3DArray):
        self.detections = msg
        self.publish_detection_markers(self.detection_markers_pub, self.detections)

    def add_detection_to_marker_array(self, marker_array, detection_3d_msg):
        cube_marker = self.make_marker(detection_3d_msg, self.marker_color)
        text_marker = self.make_marker(detection_3d_msg, self.marker_color)
        arrow_marker = self.make_marker(detection_3d_msg, self.marker_color)

        label, count = self.get_detection_label(detection_3d_msg)
        
        arrow_marker.type = Marker.ARROW
        arrow_marker.ns = "arrow_" + cube_marker.ns
        
        arrow_marker.scale.x = cube_marker.scale.x / 2.0
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05

        cube_marker.type = Marker.CUBE
        cube_marker.ns = "cube_" + cube_marker.ns

        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text_" + cube_marker.ns
        text_marker.text = "%s_%s|%0.1f" % (
            "cone",
            count,
            detection_3d_msg.results[0].score * 100,
        )
        text_marker.scale.z = min(text_marker.scale.x, text_marker.scale.y)
        # text_marker.scale.x = 0.0
        # text_marker.scale.y = 0.0

        marker_array.markers.append(cube_marker)
        marker_array.markers.append(text_marker)
        marker_array.markers.append(arrow_marker)

    def make_marker(self, detection_3d_msg, color):
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = detection_3d_msg.results[0].pose.pose
        marker.header = detection_3d_msg.header
        marker.lifetime = rospy.Duration(1.0)
        label, count = self.get_detection_label(detection_3d_msg)
        marker.ns = label
        marker.id = count

        marker.scale.x = abs(detection_3d_msg.bbox.size.x)
        marker.scale.y = abs(detection_3d_msg.bbox.size.y)
        marker.scale.z = abs(detection_3d_msg.bbox.size.z)
        marker.color = color

        return marker

    def publish_detection_markers(self, publisher: rospy.Publisher, detections: Detection3DArray):
        markers = MarkerArray()
        for detection in detections.detections:
            self.add_detection_to_marker_array(markers, detection)
        publisher.publish(markers)

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

    def run(self):
        rospy.spin()


def main():
    node = SurfaceMatchTestNode()
    node.run()


if __name__ == "__main__":
    main()
