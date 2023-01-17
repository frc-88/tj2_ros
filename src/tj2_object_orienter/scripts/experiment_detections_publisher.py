#!/usr/bin/env python3
import os
import re

import pytz
import rospy
import datetime
import numpy as np
import numpy.typing as npt
import ros_numpy

from image_geometry import PinholeCameraModel

from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, Detection3DArray, Detection3D
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock

from tj2_interfaces.msg import Labels

from tj2_tools.training.yolo import YoloFrame


def convert_pc_msg_to_np(pc_msg):
    # Fix rosbag issues, see: https://github.com/eric-wieser/ros_numpy/issues/23
    pc_msg.__class__ = PointCloud2
    offset_sorted = {f.offset: f for f in pc_msg.fields}
    pc_msg.fields = [f for (_, f) in sorted(offset_sorted.items())]

    # Conversion from PointCloud2 msg to np array.
    pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
    return pc_np


class ExperimentDetectionsPublisher:
    def __init__(self) -> None:
        rospy.init_node("experiment_detections_publisher")
        self.annotation_search_dir = "/root/tj2_ros/dataset_builder/data/charged_up_2023_raw/game_pieces"
        
        self.timezone = pytz.timezone('America/New_York')
        self.current_index = 0
        self.class_names = []
        self.cloud = np.array([])
        self.camera_model = PinholeCameraModel()
    
        self.timestamps, self.lookup_table = self.build_timestamp_mapping(self.annotation_search_dir)
        print(f"Found {len(self.timestamps)} images")
        
        self.info_sub = rospy.Subscriber("camera_info", CameraInfo, self.info_callback, queue_size=1)
        self.labels_sub = rospy.Subscriber("labels", Labels, self.labels_callback, queue_size=1)
        self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_callback, queue_size=1)
        # self.image_sub = rospy.Subscriber("image", Image, self.image_callback, queue_size=1)
        
        self.point_cloud_sub = rospy.Subscriber(
            "points",
            PointCloud2,
            self.point_cloud_callback,
            queue_size=1,
            buff_size=52428800,
        )
        
        self.detections_pub = rospy.Publisher("detections", Detection3DArray, queue_size=1)
        
    def labels_callback(self, msg):
        self.class_names = msg.labels

    def build_timestamp_mapping(self, path):
        lookup_table = {}
        timestamps = []
        
        for dirpath, dirnames, filenames in os.walk(path):
            for filename in filenames:
                if not filename.endswith(".jpg"):
                    continue
                path = os.path.join(dirpath, filename)
                match = re.search(r"image-\d+-(.+).jpg", filename)
                if not match:
                    continue
                date_string = match.group(1)
                date = datetime.datetime.strptime(date_string, "%Y-%m-%dT%H-%M-%S--%f")
                date = date.replace(tzinfo=self.timezone)
                timestamp = date.timestamp()
                timestamps.append(timestamp)
                lookup_table[timestamp] = path
        timestamps.sort()
        return timestamps, lookup_table
    
    def info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)
        # self.publish_detections(msg.header.stamp)
    
    def image_callback(self, msg):
        self.publish_detections(msg.header.stamp)
    
    def clock_callback(self, msg):
        self.publish_detections(msg.clock)
    
    def publish_detections(self, stamp):
        timestamp = stamp.to_sec()
        print(self.current_index, datetime.datetime.fromtimestamp(timestamp), datetime.datetime.fromtimestamp(min(self.timestamps)), datetime.datetime.fromtimestamp(max(self.timestamps)))
        while self.current_index < len(self.timestamps) and self.timestamps[self.current_index] < timestamp:
            self.current_index += 1
        image_path = self.lookup_table[self.timestamps[self.current_index]]
        frame_path = os.path.splitext(image_path)[0] + ".txt"
        frame = YoloFrame.from_file(frame_path, image_path, self.class_names)

        detection_2d_arr = self.get_2d_detection(stamp, frame)
        detection_3d_arr = Detection3DArray()
        detection_3d_arr.header = detection_2d_arr.header
        for detection_2d_msg in detection_2d_arr.detections:
            detection_3d_msg = self.detection_2d_to_3d(detection_2d_msg)
            detection_3d_arr.detections.append(detection_3d_msg)
        self.detections_pub.publish(detection_3d_arr)
    
    def point_cloud_callback(self, msg):
        self.cloud = convert_pc_msg_to_np(msg)
    
    def get_2d_detection(self, stamp, frame: YoloFrame) -> Detection2DArray:
        detection_arr_msg = Detection2DArray()
        detection_arr_msg.header.stamp = stamp
        for obj in frame.objects:
            detection_msg = Detection2D()
            detection_msg.header.stamp = detection_arr_msg.header.stamp
            box_width = obj.bounding_box.x1 - obj.bounding_box.x0
            box_height = obj.bounding_box.y1 - obj.bounding_box.y0
            detection_msg.bbox.center.x = obj.bounding_box.x0 + box_width / 2.0
            detection_msg.bbox.center.y = obj.bounding_box.y0 + box_height / 2.0
            detection_msg.bbox.size_x = box_width
            detection_msg.bbox.size_y = box_height
            obj_with_pose = ObjectHypothesisWithPose()
            obj_with_pose.id = self.class_names.index(obj.label)
            obj_with_pose.score = 1.0
            detection_msg.results.append(obj_with_pose)

            detection_arr_msg.detections.append(detection_msg)
        return detection_arr_msg


    def detection_2d_to_3d(self, detection_2d_msg: Detection2D) -> Detection3D:
        center_x = detection_2d_msg.bbox.center.x
        center_y = detection_2d_msg.bbox.center.y
        
        if len(self.cloud) == 0:
            return Detection3D()
        
        center_point = self.get_nearest_point(center_x, center_y)
        
        offset_x = detection_2d_msg.bbox.size_x / 2.0
        offset_y = detection_2d_msg.bbox.size_y / 2.0
        corner_point = self.get_nearest_point(center_x + offset_x, center_y + offset_y)
        
        box_size = 2.0 * (corner_point - center_point)
        
        detection_3d_msg = Detection3D()
        detection_3d_msg.header.stamp = detection_2d_msg.header.stamp
        detection_3d_msg.results = detection_2d_msg.results

        pose = Pose()
        pose.position.x = center_point[0]
        pose.position.y = center_point[1]
        pose.position.z = center_point[2]
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        detection_3d_msg.header.frame_id = self.camera_model.tfFrame()
        detection_3d_msg.results[0].pose.pose = pose
        detection_3d_msg.bbox.center = pose
        detection_3d_msg.bbox.size.x = box_size[0]
        detection_3d_msg.bbox.size.y = box_size[1]
        detection_3d_msg.bbox.size.z = box_size[2]

        return detection_3d_msg
    
    def get_nearest_point(self, x_px: float, y_px: float) -> npt.NDArray[np.float32]:
        ray = self.camera_model.projectPixelTo3dRay((x_px, y_px))
        
        # X axis for point cloud is Z axis for camera
        min_z = min(self.cloud[:, 0])
        max_z = min(self.cloud[:, 0])
        
        x0 = ray[0] * min_z
        x1 = ray[0] * max_z
        y0 = ray[1] * min_z
        y1 = ray[1] * max_z
        
        p0 = np.array([x0, y0, min_z], dtype=np.float32)
        p1 = np.array([x1, y1, max_z], dtype=np.float32)
        
        line_cross_cloud = np.cross(p1 - p0, p0 - self.cloud, axisb=1)
        distances = np.linalg.norm(line_cross_cloud, axis=1)
        norm_distances = distances / np.linalg.norm(p1 - p0)
        min_distance_indices = np.argmin(norm_distances)
        closest_point = self.cloud[min_distance_indices]
        
        return closest_point

    def run(self):
        rospy.spin()


def main():
    node = ExperimentDetectionsPublisher()
    node.run()

if __name__ == "__main__":
    main()
