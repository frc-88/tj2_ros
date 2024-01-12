#!/usr/bin/env python3
import numpy as np
import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tj2_tools.robot_state import Pose2d
from tj2_tools.transforms import lookup_transform
from tj2_tools.transforms.transform3d import Transform3D


class FieldCalibration:
    def __init__(self) -> None:
        self.speed_threshold = float(rospy.get_param("~speed_threshold", 0.05))
        self.angular_threshold = float(rospy.get_param("~angular_threshold", 0.025))
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.odom_frame = str(rospy.get_param("~odom_frame", "odom"))
        self.base_frame = str(rospy.get_param("~base_frame", "camera_link"))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.tf_broadcaster = TransformBroadcaster()
        self.is_moving = False
        self.last_check_time = rospy.Time.now()
        self.robot_pose2d = Pose2d()
        self.recorded_tags: dict[int, list[Transform3D]] = {}

        self.zed_pose = Transform3D.identity()

        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tag_callback)
        self.zed_pose_sub = rospy.Subscriber("zed_pose", PoseStamped, self.zed_pose_callback)

    def tag_callback(self, msg: AprilTagDetectionArray) -> None:
        if self.is_moving:
            rospy.logwarn("Robot is moving, ignoring tag.")
            return
        for detection in msg.detections:
            detection: AprilTagDetection
            if len(detection.id) != 1:
                rospy.logwarn(f"Ignoring tag bundle: {detection.id}")
                continue
            tag_id = detection.id[0]
            tag_pose = Transform3D.from_pose_msg(detection.pose.pose.pose)
            tag_in_map = self.transform_to_map(tag_pose)
            if tag_id not in self.recorded_tags:
                self.recorded_tags[tag_id] = []
            self.recorded_tags[tag_id].append(tag_in_map)
            rospy.loginfo(f"Recorded tag {tag_id}")
        rospy.loginfo("Data:\n" + self.get_summary())

    def get_summary(self) -> str:
        summary = ""
        for tag_id, poses in self.recorded_tags.items():
            summary += f"Tag {tag_id}: {len(poses)}\n"
        return summary

    def transform_to_map(self, pose: Transform3D) -> Transform3D:
        return pose.transform_by(self.zed_pose)

    def zed_pose_callback(self, msg: PoseStamped) -> None:
        self.zed_pose = Transform3D.from_pose_msg(msg.pose)
        self.is_moving = self.check_moving(self.zed_pose)
        self.publish_map_to_odom()

    def publish_map_to_odom(self) -> None:
        odom_tf = lookup_transform(self.tf_buffer, self.odom_frame, self.base_frame)
        if odom_tf is None:
            return
        odom_pose = Transform3D.from_msg(odom_tf.transform)
        map_to_odom_pose = self.zed_pose.transform_by(odom_pose.inverse())
        map_to_odom_tf = TransformStamped(
            header=Header(frame_id=self.map_frame, stamp=rospy.Time.now()),
            child_frame_id=self.odom_frame,
            transform=map_to_odom_pose.to_msg(),
        )
        self.tf_broadcaster.sendTransform([map_to_odom_tf])

    def check_moving(self, new_pose: Transform3D) -> bool:
        dt = (rospy.Time.now() - self.last_check_time).to_sec()

        relative_pose = new_pose.relative_to(self.zed_pose)
        speed = float(np.linalg.norm(relative_pose.position_array)) / dt

        self.last_check_time = rospy.Time.now()

        return speed > self.speed_threshold

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("field_calibration")
    FieldCalibration().run()
