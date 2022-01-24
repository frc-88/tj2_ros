#!/usr/bin/env python3

import math

import rospy
import rostopic
import rosgraph
import actionlib

import tf2_ros
import tf_conversions
import tf2_geometry_msgs

import smach
import smach_ros

import threading

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from mbf_msgs.msg import ExePathAction, ExePathGoal

from tj2_tools.particle_filter import FilterSerial


def get_topics(ns: str):
    if len(ns) > 0 and ns[0] != "/":
        ns = "/" + ns
    master = rosgraph.Master('/rostopic')
    topics = []
    pubs, subs = rostopic.get_topic_list(master=master)
    topics = [x[0] for x in subs]
    topics.extend([x[0] for x in pubs])
    return sorted(list(set(topics)))


class FollowObject:
    def __init__(self):
        self.name = "follow_object"
        rospy.init_node(
            self.name
        )
        self.exe_path_action = actionlib.SimpleActionClient("/move_base_flex/exe_path", ExePathAction)

        rospy.loginfo("Connecting to exe_path...")
        self.exe_path_action.wait_for_server()
        rospy.loginfo("exe_path connected")
        
        self.old_object_threshold = rospy.Duration(rospy.get_param("~old_object_threshold_s", 5.0))
        self.should_follow_timeout = rospy.Duration(rospy.get_param("~should_follow_timeout", 5.0))

        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.map_frame = rospy.get_param("~map_frame", "map")

        self.gameobjects_ns = rospy.get_param("~gameobjects_ns", "gameobject")
        self.future_obj_subs = {}
        for gameobjects_topic in get_topics(self.gameobjects_ns):
            subtopic = gameobjects_topic.split("/")[-1]
            if subtopic.startswith("future_"):
                subtopic_split = subtopic.split("_")
                label = subtopic_split[1]
                index = subtopic_split[2]
                serial = FilterSerial(label, index)

                subscriber = rospy.Subscriber(gameobjects_topic, PoseStamped, lambda x: self.gameobject_callback(x, serial=serial), queue_size=10)
                rospy.loginfo("%s -> %s" % (serial, gameobjects_topic))
                self.future_obj_subs[serial] = subscriber

        self.trigger_sub = rospy.Subscriber("follow_trigger", Bool, self.follow_trigger_callback, queue_size=25)
        self.should_follow = False
        self.should_follow_timer = rospy.Time.now()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.current_objects = {}

    def follow_trigger_callback(self, msg):
        self.should_follow_timeout = rospy.Time.now()
        if msg.data != self.should_follow:
            rospy.loginfo(("Enabling" if msg.data else "Disabling") + " follow object")
        self.should_follow = msg.data

    def gameobject_callback(self, msg, serial):
        self.current_objects[serial] = msg

    def exe_path_feedback(self, feedback):
        print(feedback)

    def exe_path_done(self, goal_status, result):
        print("exe path finished:", result)
    
    def get_nearest_object(self):
        min_distance = None
        min_serial = None
        for serial, pose_stamped in self.current_objects.items():
            duration = rospy.Time.now() - pose_stamped.header.stamp
            if duration > self.old_object_threshold:
                continue
            x1 = pose_stamped.pose.position.x
            y1 = pose_stamped.pose.position.y
            z1 = pose_stamped.pose.position.z
            distance = math.sqrt(x1 * x1 + y1 * y1 + z1 * z1)  # assuming pose is relative to the robot
            if min_distance is None or distance < min_distance:
                min_distance = distance
                min_serial = serial
        if min_serial is None:
            return None
        else:
            return self.current_objects[serial]
    
    def lookup_transform(self, parent_link, child_link, time_window=None, timeout=None):
        """
        Call tf_buffer.lookup_transform. Return None if the look up fails
        """
        if time_window is None:
            time_window = rospy.Time(0)
        else:
            time_window = rospy.Time.now() - time_window

        if timeout is None:
            timeout = rospy.Duration(1.0)

        try:
            return self.tf_buffer.lookup_transform(parent_link, child_link, time_window, timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (parent_link, child_link, e))
            return None
    
    def get_pose_in_map(self, pose_stamped):
        frame = pose_stamped.header.frame_id
        map_tf = self.lookup_transform(self.map_frame, frame)
        if map_tf is None:
            return None
        map_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, map_tf)
        return map_pose

    def get_robot_pose_in_map(self):
        transform = self.lookup_transform(self.map_frame, self.base_frame)
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = self.map_frame
        robot_pose.header.stamp = rospy.Time.now()
        robot_pose.pose.position.x = transform.transform.translation.x
        robot_pose.pose.position.y = transform.transform.translation.y
        robot_pose.pose.position.z = transform.transform.translation.z
        robot_pose.pose.orientation.w = transform.transform.rotation.w
        robot_pose.pose.orientation.x = transform.transform.rotation.x
        robot_pose.pose.orientation.y = transform.transform.rotation.y
        robot_pose.pose.orientation.z = transform.transform.rotation.z
        return robot_pose

    def get_path(self, pose_stamped):
        pose_map = self.get_pose_in_map(pose_stamped)
        if pose_map is None:
            return None
        robot_pose = self.get_robot_pose_in_map()
        if robot_pose is None:
            return None
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        path.poses.append(robot_pose)
        path.poses.append(pose_map)
        return path

    def run(self):
        delay = rospy.Duration(0.5)
        while not rospy.is_shutdown():
            rospy.sleep(delay)
            if not self.should_follow:
                continue
            
            goal_pose = self.get_nearest_object()
            if goal_pose is None:
                continue
            rospy.loginfo("Nearest object pose: %s" % str(goal_pose))
            path = self.get_path(goal_pose)
            if path is None:
                continue
            rospy.loginfo("Path to object: %s" % str(path))
            goal = ExePathGoal()
            goal.path = path
            goal.dist_tolerance = 0.25  # TODO set based on object size
            goal.angle_tolerance = 0.25  # TODO set based on parameters

            self.exe_path_action.cancel_goal()
            rospy.loginfo("Sending new goal")
            self.exe_path_action.send_goal(goal, feedback_cb=self.exe_path_feedback, done_cb=self.exe_path_done)



if __name__ == "__main__":
    node = FollowObject()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
