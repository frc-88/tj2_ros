#!/usr/bin/env python3

import math
import threading

import rospy
import actionlib

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist

from std_msgs.msg import Bool
from std_msgs.msg import Header

from vision_msgs.msg import Detection3DArray

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from actionlib_msgs.msg import GoalStatus

from tj2_tools.robot_state import State
from tj2_tools.transforms import lookup_transform


class MoveBasePursuitSender:
    def __init__(self):
        self.name = "move_base_pursuit_sender"
        rospy.init_node(
            self.name
        )

        self.distance_offset = rospy.get_param("~distance_offset", 0.0)
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.1)
        self.detection_timeout = rospy.Duration(rospy.get_param("~detection_timeout", 3.0))
        self.trigger_timeout = rospy.Duration(rospy.get_param("~trigger_timeout", 0.5))
        self.send_rate = rospy.get_param("~send_rate", 2.5)
        self.map_frame = rospy.get_param("~map_frame", "map")

        self.detections_sub = rospy.Subscriber("detections", Detection3DArray, self.detections_callback, queue_size=10)

        self.trigger_sub = rospy.Subscriber("follow_trigger", Bool, self.follow_trigger_callback, queue_size=10)
        self.should_follow = False
        self.should_follow_timer = rospy.Time.now()

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.follow_object_goal_pub = rospy.Publisher("follow_object_goal", PoseStamped, queue_size=10)

        self.nearest_obj = None
        self.nearest_header = Header()
        self.object_timer = rospy.Time.now()

        self.lock = threading.Lock()
        self.move_base_action = actionlib.SimpleActionClient("/pursuit/move_base", MoveBaseAction)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def follow_trigger_callback(self, msg):
        self.should_follow_timer = rospy.Time.now()
        if msg.data != self.should_follow:
            rospy.loginfo(("Enabling" if msg.data else "Disabling") + " follow object")
            if not msg.data:
                self.cancel_goal()
        self.should_follow = msg.data

    def detections_callback(self, msg):
        if not self.should_follow:
            return
        with self.lock:
            self.nearest_obj = None
            for detection in msg.detections:
                detection_pose = detection.results[0].pose.pose
                if self.nearest_obj is None or self.get_distance(detection_pose) < self.get_distance(self.nearest_obj):
                    self.nearest_obj = detection_pose
                    self.nearest_header = detection.header
            if len(msg.detections) > 0:
                self.object_timer = rospy.Time.now()
            else:
                self.nearest_obj = None
                rospy.logwarn_throttle(0.5, "No objects in detection message!")

    def get_distance(self, pose1, pose2=None):
        if pose2 is None:
            x = pose1.position.x
            y = pose1.position.y
        else:
            x1 = pose1.position.x
            y1 = pose1.position.y
            x2 = pose2.position.x
            y2 = pose2.position.y
            x = x2 - x1
            y = y2 - y1
        return math.sqrt(x * x + y * y)

    def get_heading(self, pose1, pose2=None):
        if pose2 is None:
            x = pose1.position.x
            y = pose1.position.y
        else:
            x1 = pose1.position.x
            y1 = pose1.position.y
            x2 = pose2.position.x
            y2 = pose2.position.y
            x = x2 - x1
            y = y2 - y1
        return math.atan2(y, x)

    def cancel_goal(self):
        self.move_base_action.cancel_all_goals()

    def stop_motors(self):
        action_state = self.move_base_action.get_state()
        if action_state == GoalStatus.ACTIVE:
            self.cancel_goal()
        self.cmd_vel_pub.publish(Twist())

    def get_pose_in_map(self, pose_stamped):
        frame = pose_stamped.header.frame_id
        map_tf = lookup_transform(self.tf_buffer, self.map_frame, frame)
        if map_tf is None:
            return None
        map_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, map_tf)
        return map_pose

    def pursue_object(self):
        goal_state = State(self.nearest_obj.x + self.distance_offset, self.nearest_obj.y, self.nearest_obj.heading())
        goal_pose = PoseStamped()
        goal_pose.header = self.nearest_header
        goal_pose.pose = goal_state.to_ros_pose()
        self.follow_object_goal_pub.publish(goal_pose)
        rospy.loginfo_throttle(0.25, goal_state)
        if goal_state.distance() < self.distance_threshold:
            rospy.loginfo_throttle(0.5, "Arrived at object")
            return

        global_pose = self.get_pose_in_map(goal_pose)

        pose_array = PoseArray()
        pose_array.poses.append(global_pose.pose)
        pose_array.header = global_pose.header
        goal = MoveBaseGoal()
        goal.target_poses.header.frame_id = pose_array.header.frame_id
        goal.target_poses = pose_array
        # self.cancel_goal()
        self.move_base_action.send_goal(goal)

    def run(self):
        rate = rospy.Rate(self.send_rate)
        while not rospy.is_shutdown():
            rate.sleep()
            with self.lock:
                if not self.should_follow:
                    self.stop_motors()
                    continue
                if rospy.Time.now() - self.should_follow_timer > self.trigger_timeout:
                    rospy.logwarn_throttle(0.5, "Trigger timer expired!")
                    self.stop_motors()
                    continue

                if self.nearest_obj is None:
                    rospy.logwarn_throttle(0.5, "Nearest object not set!")
                    continue

                if rospy.Time.now() - self.object_timer > self.detection_timeout:
                    continue

                self.pursue_object()

if __name__ == "__main__":
    node = MoveBasePursuitSender()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
