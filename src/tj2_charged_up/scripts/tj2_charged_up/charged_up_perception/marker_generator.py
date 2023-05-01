import rospy
import numpy as np
from typing import List, Tuple, Dict

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from charged_up_perception.detection import Detection3d


class MarkerGenerator:
    def __init__(self, color_map: Dict[str, Tuple[int, int, int]]) -> None:
        self.color_map: Dict[str, ColorRGBA] = {}
        for key, color in color_map.items():
            color_array = np.array(color) / 255.0
            self.color_map[key] = ColorRGBA(
                color_array[2], color_array[1], color_array[0], 1.0
            )

    def make_base_marker(self, frame_id, detection: Detection3d):
        pose = Pose()
        pose.position.x = float(detection.bounding_box.x)
        pose.position.y = float(detection.bounding_box.y)
        pose.position.z = float(detection.bounding_box.z)
        pose.orientation = detection.orientation

        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = pose
        marker.header.frame_id = frame_id
        marker.lifetime = rospy.Duration(1.0)  # type: ignore
        marker.ns = str(detection.label)
        marker.id = int(detection.index)
        marker.color = self.color_map[detection.label]
        return marker

    def detection_to_marker(
        self, frame_id, detection: Detection3d
    ) -> Tuple[Marker, Marker]:
        pose_marker = self.make_base_marker(frame_id, detection)
        box_marker = self.make_base_marker(frame_id, detection)

        pose_marker.type = Marker.ARROW
        pose_marker.ns += "-ARROW"
        pose_marker.scale.x = 1.0
        pose_marker.scale.y = 0.05
        pose_marker.scale.z = 0.05

        box_marker.type = Marker.CUBE
        box_marker.ns += "-CUBE"
        box_marker.scale.x = detection.bounding_box.width
        box_marker.scale.y = detection.bounding_box.height
        box_marker.scale.z = detection.bounding_box.depth
        box_marker.color.a = 0.5
        box_marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        return pose_marker, box_marker

    def detections_to_markers(
        self, frame_id: str, detections: List[Detection3d]
    ) -> MarkerArray:
        markers = MarkerArray()
        assert markers.markers is not None
        for detection in detections:
            pose_marker, box_marker = self.detection_to_marker(frame_id, detection)
            markers.markers.append(pose_marker)
            markers.markers.append(box_marker)
        return markers
