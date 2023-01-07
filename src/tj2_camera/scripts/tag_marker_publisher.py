#!/usr/bin/env python3
import copy
from typing import List

import rospy
import tf.transformations

from apriltag_ros.msg import AprilTagDetectionArray

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

from std_msgs.msg import ColorRGBA

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


class TagMarkerPublisher:
    def __init__(self):
        self.name = "tag_marker_publisher"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)
        self.tag_pose_size = 0.25
        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.marker_pub = rospy.Publisher("tag_markers", MarkerArray, queue_size=10)
        self.rotated_tag_pub = rospy.Publisher("rotated_detections", AprilTagDetectionArray, queue_size=10)

        self.tag_msg = AprilTagDetectionArray()
        self.rotate_quat = (0.5, -0.5, 0.5, 0.5)
        
        self.marker_color = (1.0, 0.0, 0.0, 1.0)

        rospy.loginfo("%s init complete" % self.name)

    def tag_callback(self, msg: AprilTagDetectionArray):
        self.tag_msg = copy.deepcopy(msg)
        for detection in msg.detections:
            pose: Pose = detection.pose.pose.pose
            detection.pose.pose.pose.orientation = self.rotate_tag_orientation(pose.orientation)
        self.rotated_tag_pub.publish(msg)
    
    def rotate_tag_orientation(self, tag_orientation: Quaternion) -> Quaternion:
        tag_quat = (
            tag_orientation.w,
            tag_orientation.x,
            tag_orientation.y,
            tag_orientation.z,
        )
        rotated_quat = tf.transformations.quaternion_multiply(tag_quat, self.rotate_quat)
        
        return Quaternion(*rotated_quat)

    def publish_marker(self, msg: AprilTagDetectionArray):
        markers = MarkerArray()
        for detection in msg.detections:
            pose: Pose = detection.pose.pose.pose
            header = detection.pose.header
            tag_id: List[int] = detection.id
            size = 0.0
            for tag_size in detection.size:
                size += tag_size
            name = "-".join([str(sub_id) for sub_id in tag_id])
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose = pose
            
            position_marker = self.make_marker(name, pose_stamped, self.marker_color)
            text_marker = self.make_marker(name, pose_stamped, self.marker_color)
            square_marker = self.make_marker(name, pose_stamped, self.marker_color)
            
            self.prep_position_marker(position_marker)
            self.prep_text_marker(text_marker, name)
            self.prep_square_marker(square_marker, size)

            markers.markers.append(position_marker)
            markers.markers.append(text_marker)
            markers.markers.append(square_marker)
        self.marker_pub.publish(markers)
    
    def prep_position_marker(self, position_marker):
        position_marker.type = Marker.ARROW
        position_marker.ns = "pos" + position_marker.ns
        position_marker.color.a = 0.75
        position_marker.scale.x = self.tag_pose_size / 4.0
        position_marker.scale.y = self.tag_pose_size / 2.5
        position_marker.scale.z = self.tag_pose_size / 2.0
        position_marker.pose = copy.deepcopy(position_marker.pose)
        position_marker.pose.orientation = self.rotate_tag_orientation(position_marker.pose.orientation)
        
        p1 = Point()
        p2 = Point()
        
        p2.x = self.tag_pose_size

        position_marker.points.append(p1)
        position_marker.points.append(p2)
    
    def prep_text_marker(self, text_marker, name):
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text" + text_marker.ns
        text_marker.text = name
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0
        text_marker.color = ColorRGBA(
            r=0.0,
            g=0.0,
            b=0.0,
            a=1.0,
        )
    
    def prep_square_marker(self, square_marker, tag_size):
        square_marker.type = Marker.CUBE
        square_marker.ns = "cube" + square_marker.ns
        square_marker.scale.x = tag_size
        square_marker.scale.y = tag_size
        square_marker.scale.z = 0.001
    
    def make_marker(self, name, pose, color):
        # name: str, marker name
        # pose: PoseStamped
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.header = pose.header
        marker.lifetime = rospy.Duration(1.0)  # seconds
        marker.ns = name
        marker.id = 0  # all waypoint names should be unique

        scale_vector = Vector3()
        scale_vector.x = self.tag_pose_size
        scale_vector.y = self.tag_pose_size
        scale_vector.z = self.tag_pose_size
        marker.scale = scale_vector
        marker.color = ColorRGBA(
            r=color[0],
            g=color[1],
            b=color[2],
            a=color[3],
        )

        return marker


    def run(self):
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            self.publish_marker(self.tag_msg)
            rate.sleep()


if __name__ == "__main__":
    node = TagMarkerPublisher()
    node.run()
