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

from std_msgs.msg import Bool
from std_msgs.msg import Header

from vision_msgs.msg import Detection3DArray

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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

        self.detections_sub = rospy.Subscriber("detections", Detection3DArray, self.detections_callback, queue_size=10)

        self.trigger_sub = rospy.Subscriber("follow_trigger", Bool, self.follow_trigger_callback, queue_size=10)
        self.should_follow = False
        self.should_follow_timer = rospy.Time.now()

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.follow_object_goal_pub = rospy.Publisher("follow_object_goal", PoseStamped, queue_size=10)

        self.delta_meas_obj = None
        self.delta_meas_robot = DeltaMeasurement()
        self.predictor = BouncePredictor(  # TODO: make dynamically configurable
            rho=0.75,
            tau=0.05,
            g=-9.81,
            a_friction=-0.1,
            t_step=0.001,
            ground_plane=-0.1,
            a_robot=5.0,
            v_max_robot=2.0,
            t_limit=10.0
        )

        self.nearest_header = Header()
        self.object_timer = rospy.Time.now()

        self.lock = threading.Lock()
        self.move_base_action = actionlib.SimpleActionClient("/pursuit/move_base", MoveBaseAction)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.move_base_goal = None
        self.goal_timer = rospy.Timer(rospy.Duration(1.0 / self.send_rate), self.send_goal_callback)

    def send_goal_callback(self):
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

    def detections_callback(self, msg):
        if not self.should_follow:
            return
        with self.lock:
            nearest_pose = None
            for detection in msg.detections:
                detection_pose = detection.results[0].pose.pose
                if nearest_pose is None or self.get_distance(detection_pose) < self.get_distance(nearest_pose.pose):
                    nearest_pose = PoseStamped()
                    nearest_pose.pose = detection_pose
                    nearest_pose.header = detection.header
            if len(msg.detections) > 0:
                self.object_timer = rospy.Time.now()
                
                # replace arbitrary object orientation with heading between object and robot
                nearest_pose.pose.orientation = self.get_heading(nearest_pose)

                # offset object by distance
                nearest_pose.pose.position.x += self.distance_offset

                # transform pose to map frame
                nearest_pose_map = self.get_pose_in_map(nearest_pose)
                if nearest_pose_map is None:
                    self.delta_meas_obj = None
                
                # compute object's velocity
                nearest_state = FilterState.from_ros_pose(nearest_pose_map.pose)
                if self.delta_meas_obj is None:
                    self.delta_meas_obj = DeltaMeasurement()
                self.delta_meas_obj.update(nearest_state)
            else:
                self.delta_meas_obj = None
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
        yaw = math.atan2(y, x)
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        quat_msg = Quaternion()
        quat_msg.w = quat[0]
        quat_msg.x = quat[1]
        quat_msg.y = quat[2]
        quat_msg.z = quat[3]
        return quat_msg

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

    def get_robot_pose(self):
        transform = lookup_transform(self.tf_buffer, self.map_frame, self.base_frame)
        if transform is None:
            return None

        robot_pose = Pose()
        robot_pose.position.x = transform.transform.translation.x
        robot_pose.position.y = transform.transform.translation.y
        robot_pose.position.z = transform.transform.translation.z
        robot_pose.orientation.w = transform.transform.rotation.w
        robot_pose.orientation.x = transform.transform.rotation.x
        robot_pose.orientation.y = transform.transform.rotation.y
        robot_pose.orientation.z = transform.transform.rotation.z
        robot_state = FilterState.from_ros_pose(robot_pose)
        return self.delta_meas_robot.update(robot_state)

    def pursue_object(self):
        if self.delta_meas_obj is None:
            return
        future_state = self.predictor.get_robot_intersection(self.get_robot_pose(), self.delta_meas_obj.state)
        future_pose = future_state.to_ros_pose()
        future_pose_stamped = PoseStamped()
        future_pose_stamped.header = self.map_frame
        future_pose_stamped.pose = future_pose
        self.follow_object_goal_pub.publish(future_pose_stamped)

        pose_array = PoseArray()
        pose_array.poses.append(future_pose_stamped)
        pose_array.header = future_pose_stamped.header
        self.move_base_goal = MoveBaseGoal()
        self.move_base_goal.target_poses.header.frame_id = pose_array.header.frame_id
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

                if self.delta_meas_obj is None:
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

