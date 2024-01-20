#!/usr/bin/env python3
import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tj2_tools.transforms import transform_pose


class FieldCalibration:
    def __init__(self) -> None:
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.base_frame = str(rospy.get_param("~base_frame", "camera_link"))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.tag_sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tag_callback, queue_size=10)
        self.filtered_tag_pub = rospy.Publisher("filtered_detections", AprilTagDetectionArray, queue_size=10)

    def tag_callback(self, msg: AprilTagDetectionArray) -> None:
        out_msg = AprilTagDetectionArray()
        out_msg.header.frame_id = self.base_frame
        for detection in msg.detections:
            detection: AprilTagDetection

            if len(detection.id) != 1:
                continue

            tag_pose_stamped = PoseStamped()
            tag_pose_stamped.header = detection.pose.header
            tag_pose_stamped.pose = detection.pose.pose.pose
            base_pose_stamped = transform_pose(
                self.tf_buffer,
                tag_pose_stamped,
                self.map_frame,
                rospy.Time.now() - detection.pose.header.stamp,
                timeout=rospy.Duration.from_sec(0.05),
            )
            if base_pose_stamped is None:
                rospy.logwarn("Failed to transform tag pose to base frame")
                continue

            detection.pose.pose.pose = base_pose_stamped.pose
            detection.pose.header.frame_id = base_pose_stamped.header.frame_id
            out_msg.detections.append(detection)

        self.filtered_tag_pub.publish(out_msg)

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("field_calibration")
    FieldCalibration().run()
