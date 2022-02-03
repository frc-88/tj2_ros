#!/usr/bin/env python3

import math

import rospy
import rostopic
import rosgraph
import actionlib

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from nav_msgs.msg import Path

from std_msgs.msg import Bool

from mbf_msgs.msg import ExePathAction, ExePathGoal

from actionlib_msgs.msg import GoalStatus

from tj2_tools.particle_filter import FilterSerial
from tj2_tools.transforms import lookup_transform


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

        self.x_goal_offset = rospy.get_param("~x_goal_offset", 0.0)
        self.y_goal_offset = rospy.get_param("~y_goal_offset", 0.0)
        self.z_goal_offset = rospy.get_param("~z_goal_offset", 0.0)

        self.gameobjects_ns = rospy.get_param("~gameobjects_ns", "gameobject")
        self.gameobject_prefix = rospy.get_param("~gameobject_prefix", "estimate_")
        self.future_obj_subs = {}
        for gameobjects_topic in get_topics(self.gameobjects_ns):
            subtopic = gameobjects_topic.split("/")[-1]
            if subtopic.startswith(self.gameobject_prefix):
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

        self.path_pub = rospy.Publisher("follow_object_path", Path, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.current_objects = {}

    def follow_trigger_callback(self, msg):
        self.should_follow_timeout = rospy.Time.now()
        if msg.data != self.should_follow:
            rospy.loginfo(("Enabling" if msg.data else "Disabling") + " follow object")
        self.should_follow = msg.data

    def gameobject_callback(self, msg, serial):
        obj_pose = PoseStamped()
        obj_pose.header = msg.header
        obj_pose.pose.position.x = msg.pose.position.x + self.x_goal_offset
        obj_pose.pose.position.y = msg.pose.position.y + self.y_goal_offset
        obj_pose.pose.position.z = msg.pose.position.z + self.z_goal_offset
        obj_pose.pose.orientation = msg.pose.orientation
        self.current_objects[serial] = obj_pose

    def stop_motors(self):
        rospy.loginfo("Stopping motors")
        self.cmd_vel_pub.publish(Twist())

    def exe_path_feedback(self, feedback):
        print(feedback.message)

    def exe_path_done(self, goal_status, result):
        print("exe path finished:", result.message)
        self.stop_motors()
    
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

    def get_pose_in_map(self, pose_stamped):
        frame = pose_stamped.header.frame_id
        map_tf = lookup_transform(self.tf_buffer, self.map_frame, frame)
        if map_tf is None:
            return None
        map_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, map_tf)
        return map_pose

    def get_robot_pose_in_map(self):
        transform = lookup_transform(self.tf_buffer, self.map_frame, self.base_frame)
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
                action_state = self.exe_path_action.get_state()
                if action_state == GoalStatus.ACTIVE:
                    rospy.loginfo("Cancelling exe path")
                    self.exe_path_action.cancel_all_goals()
                continue
            
            goal_pose = self.get_nearest_object()
            if goal_pose is None:
                continue
            rospy.loginfo("Nearest object pose: %s" % str(goal_pose))
            path = self.get_path(goal_pose)
            self.path_pub.publish(path)
            if path is None:
                continue
            # rospy.loginfo("Path to object: %s" % str(path))
            goal = ExePathGoal()
            goal.path = path
            goal.dist_tolerance = 0.25  # TODO set based on object size
            goal.angle_tolerance = 0.25  # TODO set based on parameters

            self.exe_path_action.cancel_all_goals()
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
