#!/usr/bin/env python3
import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from tj2_tools.robot_state import Pose2d
from tj2_tools.transforms import transform_pose


class FieldCalibration:
    def __init__(self) -> None:
        self.speed_threshold = float(rospy.get_param("~speed_threshold", 0.05))
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tag_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.is_moving = False
        self.last_check_time = rospy.Time.now()
        self.robot_pose2d = Pose2d()
        self.recorded_tags: dict[int, list[PoseStamped]] = {}

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
            tag_pose = PoseStamped(header=detection.pose.header, pose=detection.pose.pose.pose)
            tag_in_map = transform_pose(self.tf_buffer, tag_pose, self.map_frame)
            if tag_in_map is None:
                rospy.logwarn(f"Could not transform tag {tag_id} to map frame")
                continue
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

    def check_moving(self) -> bool:
        pose_stamped = PoseStamped(
            header=Header(frame_id="base_link"), pose=Pose(position=Point(), orientation=Quaternion(w=1.0))
        )
        robot_pose = transform_pose(self.tf_buffer, pose_stamped, self.map_frame)
        if robot_pose is None:
            return True
        dt = (rospy.Time.now() - self.last_check_time).to_sec()

        robot_pose2d = Pose2d.from_ros_pose(robot_pose.pose)
        relative_pose = robot_pose2d.relative_to(self.robot_pose2d)
        speed = relative_pose.magnitude() / dt

        self.last_check_time = rospy.Time.now()

        return speed > self.speed_threshold

    def run(self) -> None:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.is_moving = self.check_moving()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("field_calibration")
    FieldCalibration().run()
