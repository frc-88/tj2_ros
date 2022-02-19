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

from nav_msgs.msg import Odometry

from vision_msgs.msg import Detection3DArray

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber

from actionlib_msgs.msg import GoalStatus

from tj2_pursuit.msg import PursueObjectAction, PursueObjectGoal, PursueObjectResult

from tj2_tools.particle_filter.state import FilterState
from tj2_tools.predictions.predictor import BouncePredictor
from tj2_tools.transforms import lookup_transform
from tj2_tools.yolo.utils import get_label, read_class_names


class Tj2Pursuit:
    def __init__(self):
        self.name = "tj2_pursuit"
        rospy.init_node(
            self.name
        )

        self.distance_offset = rospy.get_param("~distance_offset", 0.0)
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.1)
        self.send_rate = rospy.get_param("~send_rate", 2.5)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.class_names_path = rospy.get_param("~class_names_path", "objects.names")
        self.class_names = read_class_names(self.class_names_path)

        self.detections_sub = Subscriber("detections", Detection3DArray)
        self.odom_sub = Subscriber("odom", Odometry)
        self.time_sync = ApproximateTimeSynchronizer([self.detections_sub, self.odom_sub], queue_size=50, slop=0.02)
        self.time_sync.registerCallback(self.obj_odom_callback)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.follow_object_goal_pub = rospy.Publisher("follow_object_goal", PoseStamped, queue_size=10)

        self.tracking_object_name = ""
        
        self.predictor = BouncePredictor(  # TODO: make dynamically configurable
            v_max_robot=4.0,
            past_window_size=4,
            vx_std_dev_threshold=1.0,
            vy_std_dev_threshold=1.0
        )
        self.future_pose_stamped = None
        self.prev_pose_stamped = None

        self.lock = threading.Lock()
        self.move_base_action = actionlib.SimpleActionClient("/pursuit/move_base", MoveBaseAction)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pursue_object_server = actionlib.SimpleActionServer("pursue_object", PursueObjectAction, self.pursue_object_callback, auto_start=False)
        self.pursue_object_server.start()
        rospy.loginfo("%s is ready" % self.name)

    def pursue_object_callback(self, goal):
        rate = rospy.Rate(self.send_rate)
        start_timer = rospy.Time.now()
        detection_timeout = goal.timeout
        self.tracking_object_name = goal.object_name
        self.prev_pose_stamped = None
        while True:
            rate.sleep()
            with self.lock:
                if rospy.Time.now() - start_timer > detection_timeout:
                    self.pursue_object_server.set_aborted()
                    rospy.logwarn("Timed out while searching for '%s'" % self.tracking_object_name)
                    break
                if rospy.is_shutdown() or self.pursue_object_server.is_preempt_requested():
                    self.pursue_object_server.set_preempted()
                    break

                if self.pursue_object(goal.xy_tolerance):
                    self.pursue_object_server.set_succeeded()
                    break

        self.cancel_goal()

    def obj_odom_callback(self, detections_msg, odom_msg):
        # if len(self.tracking_object_name) == 0:
        #     return
        with self.lock:
            nearest_obj = self.get_nearest_detection(self.tracking_object_name, detections_msg)
            self.future_pose_stamped = None
            if nearest_obj is None:
                return
            odom_state = FilterState.from_odom(odom_msg)
            object_state = nearest_obj.relative_to(odom_state)
            predicted_state = self.predictor.get_robot_intersection(object_state, odom_state)
            if predicted_state is not None:
                self.object_timer = rospy.Time.now()
                future_pose_stamped = PoseStamped()
                future_pose_stamped.pose = predicted_state.to_ros_pose()
                future_pose_stamped.header = odom_msg.header
                self.follow_object_goal_pub.publish(future_pose_stamped)
                if len(self.tracking_object_name) != 0:
                    self.future_pose_stamped = future_pose_stamped

    def get_nearest_detection(self, object_name, detections_msg):
        nearest_pose = None
        nearest_dist = None
        for detection in detections_msg.detections:
            if len(object_name) > 0:
                label, index = get_label(self.class_names, detection.results[0].id)
                if label != object_name:
                    continue
            detection_pose = detection.results[0].pose.pose
            detection_dist = self.get_distance(detection_pose)
            if nearest_dist is None or detection_dist < nearest_dist:
                nearest_pose = PoseStamped()
                nearest_pose.pose = detection_pose
                nearest_pose.header = detection.header
                nearest_dist = detection_dist
        
        if nearest_pose is not None:
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
        self.tracking_object_name = ""
        self.move_base_action.cancel_all_goals()
        self.stop_motors()

    def stop_motors(self):
        self.cmd_vel_pub.publish(Twist())

    def get_pose_in_map(self, pose_stamped):
        frame = pose_stamped.header.frame_id
        map_tf = lookup_transform(self.tf_buffer, self.map_frame, frame)
        if map_tf is None:
            return None
        map_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, map_tf)
        return map_pose

    def get_robot_pose(self):
        map_tf = lookup_transform(self.tf_buffer, self.map_frame, self.base_frame)
        if map_tf is None:
            return None
        map_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(), map_tf)
        return map_pose

    def pursue_object(self, xy_tolerance):
        if self.future_pose_stamped is None:
            rospy.logwarn("No prediction is available to pursue!")
            return False
        future_map_pose_stamped = self.get_pose_in_map(self.future_pose_stamped)
        robot_pose_stamped = self.get_robot_pose()
        distance = self.get_distance(future_map_pose_stamped.pose, robot_pose_stamped.pose)
        if distance < xy_tolerance:
            rospy.loginfo("Object reached!")
            return True
        if self.prev_pose_stamped == future_map_pose_stamped:
            return False
        self.prev_pose_stamped = future_map_pose_stamped

        rospy.loginfo("Sending goal prediction for %s" % self.tracking_object_name)

        pose_array = PoseArray()
        pose_array.poses.append(future_map_pose_stamped.pose)
        pose_array.header = future_map_pose_stamped.header
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_poses = pose_array
        self.move_base_action.send_goal(move_base_goal)
        return False

    def run(self):
        rospy.spin()
    
if __name__ == "__main__":
    node = Tj2Pursuit()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)

