#!/usr/bin/env python3

import math
import threading

import rospy
import actionlib

import tf2_ros
import tf2_geometry_msgs

from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion

from nav_msgs.msg import Odometry

from std_msgs.msg import Bool
from std_msgs.msg import Header

from vision_msgs.msg import Detection3DArray

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber

from actionlib_msgs.msg import GoalStatus

from tj2_tools.particle_filter.state import FilterState
from tj2_tools.particle_filter.state import DeltaMeasurement
from tj2_tools.particle_filter.predictor import BouncePredictor
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
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.detections_sub = Subscriber("detections", Detection3DArray, self.detections_callback, queue_size=10)
        self.odom_sub = Subscriber("odom", Odometry, self.odom_callback, queue_size=25)
        self.time_sync = ApproximateTimeSynchronizer([self.detections_sub, self.odom_sub], queue_size=50, slop=0.02)
        self.time_sync.registerCallback(self.obj_odom_callback)

        self.trigger_sub = rospy.Subscriber("follow_trigger", Bool, self.follow_trigger_callback, queue_size=10)
        self.should_follow = False
        self.should_follow_timer = rospy.Time.now()

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.follow_object_goal_pub = rospy.Publisher("follow_object_goal", PoseStamped, queue_size=10)

        self.predictor = BouncePredictor(  # TODO: make dynamically configurable
            v_max_robot=4.0,
            past_window_size=4,
            vx_std_dev_threshold=1.0,
            vy_std_dev_threshold=1.0
        )
        self.future_pose_stamped = PoseStamped()

        self.object_timer = rospy.Time.now()

        self.lock = threading.Lock()
        self.move_base_action = actionlib.SimpleActionClient("/pursuit/move_base", MoveBaseAction)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.move_base_goal = None
        self.goal_timer = rospy.Timer(rospy.Duration(1.0 / self.send_rate), self.send_goal_callback)

    def send_goal_callback(self, timer):
        if self.move_base_goal is None:
            return
        self.move_base_action.send_goal(self.move_base_goal)

    def follow_trigger_callback(self, msg):
        self.should_follow_timer = rospy.Time.now()
        if msg.data != self.should_follow:
            rospy.loginfo(("Enabling" if msg.data else "Disabling") + " follow object")
            if not msg.data:
                self.cancel_goal()
        self.should_follow = msg.data

    def obj_odom_callback(self, detections_msg, odom_msg):
        with self.lock:
            nearest_obj = self.get_nearest_detection(detections_msg)
            if nearest_obj is None:
                return
            odom_state = FilterState.from_odom(odom_msg)
            object_state = nearest_obj.relative_to(odom_state)
            predicted_state = self.predictor.get_robot_intersection(object_state, odom_state)
            if predicted_state is not None:
                self.object_timer = rospy.Time.now()

                self.future_pose_stamped.pose = predicted_state.to_ros_pose()
                self.future_pose_stamped.header = odom_msg.header

    def get_nearest_detection(self, detections_msg):
        nearest_pose = None
        nearest_dist = None
        for detection in detections_msg.detections:
            detection_pose = detection.results[0].pose.pose
            detection_dist = self.get_distance(detection_pose)
            if nearest_dist is None or detection_dist < nearest_dist:
                nearest_pose = PoseStamped()
                nearest_pose.pose = detection_pose
                nearest_pose.header = detection.header
                nearest_dist = detection_dist
        
        if nearest_pose is not None:
            # compute object's velocity
            nearest_state = FilterState.from_ros_pose(nearest_pose.pose)
            nearest_state.stamp = nearest_pose.header.stamp.to_sec()
            return nearest_state
        else:
            return None

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

    def cancel_goal(self):
        self.move_base_action.cancel_all_goals()
        self.move_base_goal = None

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
        self.follow_object_goal_pub.publish(self.future_pose_stamped)

        pose_array = PoseArray()
        pose_array.poses.append(self.future_pose_stamped.pose)
        pose_array.header = self.future_pose_stamped.header
        self.move_base_goal = MoveBaseGoal()
        self.move_base_goal.target_poses = pose_array

    def run(self):
        rate = rospy.Rate(10.0)
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

