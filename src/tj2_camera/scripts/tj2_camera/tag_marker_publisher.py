#!/usr/bin/env python3
import copy
from typing import List, Optional, Tuple

import rospy
import tf2_ros
import tf2_geometry_msgs

from scipy.spatial.transform import Rotation

from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Pose, Point, Vector3, PoseStamped, Quaternion, PoseArray
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker


class TagMarkerPublisher:
    def __init__(self):
        self.name = "tag_marker_publisher"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)
        self.tag_pose_marker_size = rospy.get_param("~tag_pose_marker_size", 0.25)
        self.marker_publish_rate = rospy.get_param("~marker_publish_rate", 5.0)
        self.debug = rospy.get_param("~debug", False)
        self.robot_frame = rospy.get_param("~robot_frame", "base_link")
        self.stale_detection_seconds = rospy.Duration(
            rospy.get_param("~stale_detection_seconds", 1.0)
        )

        self.tag_msg = AprilTagDetectionArray()
        self.rotate_quat = (0.5, -0.5, -0.5, -0.5)

        self.marker_colors = {
            "1": (1.0, 0.0, 0.0, 1.0),
            "2": (1.0, 0.0, 0.0, 1.0),
            "3": (1.0, 0.0, 0.0, 1.0),
            "4": (0.25, 0.25, 1.0, 1.0),
            "5": (1.0, 0.25, 0.25, 1.0),
            "6": (0.0, 0.0, 1.0, 1.0),
            "7": (0.0, 0.0, 1.0, 1.0),
            "8": (0.0, 0.0, 1.0, 1.0),
            None: (1.0, 1.0, 1.0, 1.0),
        }

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.tag_sub = rospy.Subscriber(
            "tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10
        )
        self.marker_pub = rospy.Publisher("tag_markers", MarkerArray, queue_size=10)
        self.rotated_tag_pub = rospy.Publisher(
            "rotated_detections", AprilTagDetectionArray, queue_size=10
        )
        self.rotated_debug_pub = rospy.Publisher(
            "rotated_debug", PoseArray, queue_size=10
        )

        rospy.loginfo("%s init complete" % self.name)

    def tag_callback(self, msg: AprilTagDetectionArray):
        if not self.is_topic_active():
            return
        base_detections = AprilTagDetectionArray()
        for detection in msg.detections:
            if len(detection.id) == 1:
                pose: Pose = detection.pose.pose.pose
                detection.pose.pose.pose.orientation = self.rotate_tag_orientation(
                    pose.orientation, self.rotate_quat
                )
            new_detection = self.transform_tag_to_base(detection)
            if new_detection is None:
                continue
            base_detections.header = new_detection.pose.header
            base_detections.detections.append(new_detection)

        if len(base_detections.detections) != 0:
            self.tag_msg = base_detections
        self.rotated_tag_pub.publish(self.tag_msg)
        if self.debug:
            self.publish_debug_rotation(self.tag_msg)

    def is_topic_active(self):
        num_subscriptions = (
            self.marker_pub.get_num_connections()
            + self.rotated_tag_pub.get_num_connections()
            + self.rotated_debug_pub.get_num_connections()
        )
        return num_subscriptions > 0

    def publish_debug_rotation(self, msg: AprilTagDetectionArray):
        pose_array = PoseArray()
        pose_array.header = msg.header
        for detection in msg.detections:
            pose: Pose = detection.pose.pose.pose
            pose_array.poses.append(pose)
        self.rotated_debug_pub.publish(pose_array)

    def transform_tag_to_base(
        self, detection: AprilTagDetection
    ) -> Optional[AprilTagDetection]:
        tag_pose_stamped = PoseStamped()
        tag_pose_stamped.header = detection.pose.header
        tag_pose_stamped.pose = detection.pose.pose.pose

        if self.robot_frame == tag_pose_stamped.header.frame_id:
            return tag_pose_stamped
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                tag_pose_stamped.header.frame_id,
                rospy.Time(0),
                self.stale_detection_seconds,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn(
                "Failed to look up %s to %s. %s"
                % (self.robot_frame, tag_pose_stamped.header.frame_id, e)
            )
            return None
        base_pose_stamped = tf2_geometry_msgs.do_transform_pose(
            tag_pose_stamped, transform
        )

        new_detection = copy.deepcopy(detection)
        new_detection.pose.pose.pose = base_pose_stamped.pose
        new_detection.pose.header = base_pose_stamped.header

        return new_detection

    def rotate_tag_orientation(
        self,
        tag_orientation: Quaternion,
        rotate_quat: Tuple[float, float, float, float],
    ) -> Quaternion:
        rotate_mat = Rotation.from_quat(rotate_quat)  # type: ignore
        tag_mat = Rotation.from_quat(
            (  # type: ignore
                tag_orientation.x,
                tag_orientation.y,
                tag_orientation.z,
                tag_orientation.w,
            )
        )
        rotated_tag = tag_mat * rotate_mat
        return Quaternion(*rotated_tag.as_quat())

    def publish_marker(self, msg: AprilTagDetectionArray):
        if not self.is_topic_active():
            return
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

            if name in self.marker_colors:
                marker_color = self.marker_colors[name]
            else:
                marker_color = self.marker_colors[None]
            x_position_marker = self.make_marker(name, pose_stamped, marker_color)
            y_position_marker = self.make_marker(name, pose_stamped, marker_color)
            z_position_marker = self.make_marker(name, pose_stamped, marker_color)
            text_marker = self.make_marker(name, pose_stamped, marker_color)
            square_marker = self.make_marker(name, pose_stamped, marker_color)

            self.prep_position_marker("x", x_position_marker, None, (1.0, 0.0, 0.0))
            self.prep_position_marker(
                "y",
                y_position_marker,
                (0.0000, 0.0000, 0.7071, 0.7071),
                (0.0, 1.0, 0.0),
            )
            self.prep_position_marker(
                "z",
                z_position_marker,
                (0.0000, -0.7071, 0.0000, 0.7071),
                (0.0, 0.0, 1.0),
            )
            self.prep_text_marker(text_marker, name)
            self.prep_square_marker(square_marker, size)

            markers.markers.append(x_position_marker)
            markers.markers.append(y_position_marker)
            markers.markers.append(z_position_marker)
            markers.markers.append(text_marker)
            if len(detection.id) == 1:
                markers.markers.append(square_marker)
        self.marker_pub.publish(markers)

    def prep_position_marker(
        self, name, position_marker, rotate_quat, color: Tuple[float, float, float]
    ):
        position_marker.type = Marker.ARROW
        position_marker.ns = "pos" + name + position_marker.ns
        position_marker.color.r = color[0]
        position_marker.color.g = color[1]
        position_marker.color.b = color[2]
        position_marker.color.a = 0.75
        position_marker.scale.x = self.tag_pose_marker_size / 4.0
        position_marker.scale.y = self.tag_pose_marker_size / 2.5
        position_marker.scale.z = self.tag_pose_marker_size / 2.0
        if rotate_quat is not None:
            position_marker.pose.orientation = self.rotate_tag_orientation(
                position_marker.pose.orientation, rotate_quat
            )

        p1 = Point()
        p2 = Point()

        p2.x = self.tag_pose_marker_size

        position_marker.points.append(p1)
        position_marker.points.append(p2)

    def prep_text_marker(self, text_marker, name):
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text" + text_marker.ns
        text_marker.text = name
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0
        text_marker.color = ColorRGBA(
            r=1.0,
            g=1.0,
            b=1.0,
            a=1.0,
        )

    def prep_square_marker(self, square_marker, tag_size):
        square_marker.type = Marker.CUBE
        square_marker.ns = "cube" + square_marker.ns
        square_marker.scale.x = 0.001
        square_marker.scale.y = tag_size
        square_marker.scale.z = tag_size

    def make_marker(self, name, pose, color):
        # name: str, marker name
        # pose: PoseStamped
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(pose.pose)
        marker.header = pose.header
        marker.lifetime = rospy.Duration(2.0 / self.marker_publish_rate)
        marker.ns = name
        marker.id = 0  # all waypoint names should be unique

        scale_vector = Vector3()
        scale_vector.x = self.tag_pose_marker_size
        scale_vector.y = self.tag_pose_marker_size
        scale_vector.z = self.tag_pose_marker_size
        marker.scale = scale_vector
        marker.color = ColorRGBA(
            r=color[0],
            g=color[1],
            b=color[2],
            a=color[3],
        )

        return marker

    def run(self):
        rate = rospy.Rate(self.marker_publish_rate)
        while not rospy.is_shutdown():
            self.publish_marker(self.tag_msg)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                continue


if __name__ == "__main__":
    node = TagMarkerPublisher()
    node.run()
