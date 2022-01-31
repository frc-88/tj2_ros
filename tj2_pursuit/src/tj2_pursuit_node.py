#!/usr/bin/env python3

import math

import threading

import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from std_msgs.msg import Bool
from std_msgs.msg import Header

from vision_msgs.msg import Detection3DArray

from tj2_tools.robot_state import State


class Tj2Pursuit:
    def __init__(self):
        self.name = "follow_object"
        rospy.init_node(
            self.name
        )

        self.distance_offset = rospy.get_param("~distance_offset", 0.0)
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.1)
        self.detection_timeout = rospy.Duration(rospy.get_param("~detection_timeout", 3.0))
        self.trigger_timeout = rospy.Duration(rospy.get_param("~trigger_timeout", 0.5))
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.5)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.5)
        self.x_kP = rospy.get_param("~x_kP", 1.0)
        self.y_kP = rospy.get_param("~y_kP", 1.0)
        self.theta_kP = rospy.get_param("~theta_kP", 1.0)
        self.sentry_angular_velocity = rospy.get_param("~sentry_angular_velocity", 0.0)

        assert self.max_linear_speed >= 0.0, self.max_linear_speed
        assert self.max_angular_speed >= 0.0, self.max_angular_speed

        self.detections_sub = rospy.Subscriber("detections", Detection3DArray, self.detections_sub, queue_size=10)

        self.trigger_sub = rospy.Subscriber("follow_trigger", Bool, self.follow_trigger_callback, queue_size=10)
        self.should_follow = False
        self.should_follow_timer = rospy.Time.now()

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.follow_object_goal_pub = rospy.Publisher("follow_object_goal", PoseStamped, queue_size=10)

        self.nearest_obj = None
        self.nearest_header = Header()
        self.object_timer = rospy.Time.now()

        self.lock = threading.Lock()

    def follow_trigger_callback(self, msg):
        self.should_follow_timer = rospy.Time.now()
        if msg.data != self.should_follow:
            rospy.loginfo(("Enabling" if msg.data else "Disabling") + " follow object")
        self.should_follow = msg.data

    def detections_sub(self, msg):
        if not self.should_follow:
            return
        with self.lock:
            self.nearest_obj = None
            for detection in msg.detections:
                detection_pose = detection.results[0].pose.pose
                obj_state = State.from_ros_pose(detection_pose)
                if self.nearest_obj is None or obj_state.distance() < self.nearest_obj.distance():
                    self.nearest_obj = obj_state
                    self.nearest_header = detection.header
            if len(msg.detections) > 0:
                self.object_timer = rospy.Time.now()
            else:
                self.nearest_obj = None
                rospy.logwarn_throttle(0.5, "No objects in detection message!")

    def stop_motors(self):
        self.cmd_vel_pub.publish(Twist())

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

        vx = self.x_kP * goal_state.x
        vy = self.y_kP * goal_state.y
        vt = self.theta_kP * goal_state.theta
        if abs(vx) > self.max_linear_speed:
            vx = math.copysign(self.max_linear_speed, vx)
        if abs(vy) > self.max_linear_speed:
            vy = math.copysign(self.max_linear_speed, vy)
        if abs(vt) > self.max_angular_speed:
            vt = math.copysign(self.max_angular_speed, vt)

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = vt
        self.cmd_vel_pub.publish(twist)

    def run_sentry_mode(self):
        if self.sentry_angular_velocity == 0.0:
            self.stop_motors()
        else:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = self.sentry_angular_velocity
            self.cmd_vel_pub.publish(twist)

    def run(self):
        rate = rospy.Rate(30.0)
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
                    self.run_sentry_mode()
                    continue

                if rospy.Time.now() - self.object_timer > self.detection_timeout:
                    rospy.logwarn_throttle(0.5, "Object timer expired!")
                    self.run_sentry_mode()
                    continue

                self.pursue_object()

if __name__ == "__main__":
    node = Tj2Pursuit()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
