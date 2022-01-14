#!/usr/bin/env python3
import traceback
import math

import rospy
import actionlib
import tf2_ros

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Bool
from std_msgs.msg import Int32

from move_base_msgs.msg import MoveBaseAction

import tf2_geometry_msgs

from tj2_tools.joystick import Joystick

from tj2_driver_station.srv import SetRobotMode
from tj2_driver_station.msg import RobotStatus

from vision_msgs.msg import Detection2DArray


class TJ2DebugJoystick:
    SEND_AUTO_PLAN = 1
    RESET_IMU = 2

    def __init__(self):
        rospy.init_node(
            "tj2_debug_joystick",
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.twist_command = Twist()
        self.twist_command.linear.x = 0.0
        self.twist_command.linear.y = 0.0
        self.twist_command.angular.z = 0.0

        self.cmd_vel_timer = rospy.Time.now()
        self.disable_timer = rospy.Time.now()

        self.cmd_vel_timeout = rospy.Duration(0.5)
        self.send_timeout = rospy.Duration(self.cmd_vel_timeout.to_sec() + 1.0)
        self.disable_timeout = rospy.Duration(30.0)

        assert self.send_timeout.to_sec() > self.cmd_vel_timeout.to_sec()

        # parameters from launch file
        self.linear_x_axis = rospy.get_param("~linear_x_axis", "left/X").split("/")
        self.linear_y_axis = rospy.get_param("~linear_y_axis", "left/Y").split("/")
        self.angular_axis = rospy.get_param("~angular_axis", "right/X").split("/")
        self.idle_axis = rospy.get_param("~idle_axis", "brake/L").split("/")
        self.speed_selector_axis = rospy.get_param("~speed_selector_axis", "dpad/vertical").split("/")
        self.follow_object_axis = rospy.get_param("~follow_object_axis", "brake/L").split("/")

        self.linear_x_scale_max = float(rospy.get_param("~linear_x_scale", 1.0))
        self.linear_y_scale_max = float(rospy.get_param("~linear_y_scale", 1.0))
        self.angular_scale_max = float(rospy.get_param("~angular_scale", 1.0))
        self.linear_x_scale = self.linear_x_scale_max
        self.linear_y_scale = self.linear_y_scale_max
        self.angular_scale = self.angular_scale_max

        self.deadzone_joy_val = float(rospy.get_param("~deadzone_joy_val", 0.05))
        self.joystick_topic = rospy.get_param("~joystick_topic", "/joy")

        self.button_mapping = rospy.get_param("~button_mapping", None)
        assert self.button_mapping is not None
        self.axis_mapping = rospy.get_param("~axis_mapping", None)
        assert self.axis_mapping is not None

        self.global_frame = rospy.get_param("~global_frame", "map")

        self.speed_mode = 0
        self.speed_multipliers = [0.1, 0.25, 0.5, 0.9, 1.0]
        self.set_speed_mode(0)

        self.is_field_relative = False
        self.limelight_led_mode = False  # False == on, True == off

        self.joystick = Joystick(self.button_mapping, self.axis_mapping)

        self.move_base_goals = PoseArray()
        self.should_send_goals = False

        # services
        self.set_robot_mode = rospy.ServiceProxy("robot_mode", SetRobotMode)
        self.last_set_mode_time = rospy.Time.now()

        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        # rospy.loginfo("Connecting to move_base...")
        # self.move_base.wait_for_server()
        # rospy.loginfo("move_base connected")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
        self.set_field_relative_pub = rospy.Publisher("set_field_relative", Bool, queue_size=100)
        self.limelight_led_pub = rospy.Publisher("/limelight/led_mode", Bool, queue_size=5)
        self.debug_cmd_pub = rospy.Publisher("debug_cmd", Int32, queue_size=5)
        self.move_base_simple_pub = rospy.Publisher("/move_base_simple/waypoints", PoseArray, queue_size=5)

        # subscription topics
        self.joy_sub = rospy.Subscriber(self.joystick_topic, Joy, self.joystick_msg_callback, queue_size=5)
        self.detections_sub = rospy.Subscriber("/tj2/tj2_2020/detections", Detection2DArray, self.detections_callback, queue_size=5)

        rospy.Timer(rospy.Duration(0.25), self.send_goals_timer)

        rospy.loginfo("Debug joystick is ready!")

    def detections_callback(self, msg):
        self.move_base_goals.poses = []
        self.move_base_goals.header.frame_id = self.global_frame
        
        transform = self.lookup_transform(self.global_frame, msg.detections[0].header.frame_id)
        if transform is None:
            return
        
        for detection in msg.detections:
            object_pose_stamped = PoseStamped()
            object_pose_stamped.header = detection.header
            object_pose_stamped.pose = detection.results[0].pose.pose

            yaw = euler_from_quaternion([
                object_pose_stamped.pose.orientation.x,
                object_pose_stamped.pose.orientation.y,
                object_pose_stamped.pose.orientation.z,
                object_pose_stamped.pose.orientation.w,
            ])[2]

            quat = quaternion_from_euler(0.0, 0.0, yaw)
            object_pose_stamped.pose.orientation.x = quat[0]
            object_pose_stamped.pose.orientation.y = quat[1]
            object_pose_stamped.pose.orientation.z = quat[2]
            object_pose_stamped.pose.orientation.w = quat[3]

            move_base_goal_stamped = tf2_geometry_msgs.do_transform_pose(object_pose_stamped, transform)

            self.move_base_goals.poses.append(move_base_goal_stamped.pose)

    def send_goals_timer(self, event):
        if not self.should_send_goals:
            return
        if len(self.move_base_goals.poses) == 0:
            rospy.logwarn_throttle(1.0, "No goals to send!")
            return
        self.move_base_simple_pub.publish(self.move_base_goals)

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
    
    def joystick_msg_callback(self, msg):
        """
        If the joystick disconnects, this callback will stop being called.
        If a non-zero twist is set, the command timer will be reset
            This way, if the joystick is idle, it will stop publishing after send timer is exceeded
        If the command timer is exceeded in the main loop, the twist command will be set to zero
            If the joystick disconnects (i.e. this callback stops being called), the twist command will
            quickly be set to zero
        If the send timer is exceeded in the main loop, this node will stop publishing twist
        Note: the idle axis (one of the triggers) can be pressed to keep the command timer active so the robot doesn't disable itself
        """

        self.joystick.update(msg)
        
        if all(self.joystick.check_list(self.joystick.is_button_down, ("triggers", "L1"), ("menu", "Start"))):
            self.set_mode(RobotStatus.TELEOP)
        elif all(self.joystick.check_list(self.joystick.is_button_down, ("menu", "Select"), ("menu", "Start"))):
            self.set_mode(RobotStatus.AUTONOMOUS)
        elif any(self.joystick.check_list(self.joystick.did_button_down, ("triggers", "L1"), ("triggers", "R1"))):
            self.set_mode(RobotStatus.DISABLED)
        elif self.joystick.did_button_down(("main", "B")):
            self.set_field_relative(not self.is_field_relative)
        elif self.joystick.did_button_down(("main", "A")):
            self.limelight_led_mode = not self.limelight_led_mode
            rospy.loginfo("Setting limelight led mode to %s" % self.limelight_led_mode)
            self.limelight_led_pub.publish(self.limelight_led_mode)
        elif self.joystick.did_button_down(("main", "Y")):
            msg = Int32()
            msg.data = self.SEND_AUTO_PLAN
            rospy.loginfo("Requesting auto plan be sent")
            self.debug_cmd_pub.publish(msg)
        elif self.joystick.did_button_down(("main", "X")):
            msg = Int32()
            msg.data = self.RESET_IMU
            rospy.loginfo("Requesting IMU be reset")
            self.debug_cmd_pub.publish(msg)

        if any(self.joystick.check_list(self.joystick.did_axis_change, self.linear_x_axis, self.linear_y_axis, self.angular_axis)):
            self.disable_timer = rospy.Time.now()
            linear_x_val = self.joystick.deadband_axis(self.linear_x_axis, self.deadzone_joy_val, self.linear_x_scale)
            linear_y_val = self.joystick.deadband_axis(self.linear_y_axis, self.deadzone_joy_val, self.linear_y_scale)
            angular_val = self.joystick.deadband_axis(self.angular_axis, self.deadzone_joy_val, self.angular_scale)

            self.set_twist(linear_x_val, linear_y_val, angular_val)
        
        if (self.joystick.did_axis_change(self.idle_axis) or
                self.twist_command.linear.x != 0.0 or 
                self.twist_command.linear.y != 0.0 or 
                self.twist_command.angular.z != 0.0):
            self.cmd_vel_timer = rospy.Time.now()
        
        if self.joystick.did_axis_change(self.speed_selector_axis):
            axis_value = self.joystick.get_axis(self.speed_selector_axis)
            if axis_value > 0:
                self.set_speed_mode(self.speed_mode + 1)
            elif axis_value < 0:
                self.set_speed_mode(self.speed_mode - 1)
        
        if self.joystick.did_axis_change(self.follow_object_axis):
            axis_value = self.joystick.get_axis(self.follow_object_axis)
            if axis_value < 0.5:  # brake trigger is pressed
                # send detection goals to move_base
                self.should_send_goals = True
            elif axis_value > -0.5: # brake trigger is released
                # cancel move_base goal
                if self.should_send_goals:
                    self.should_send_goals = False
                    self.move_base.cancel_all_goals()
                    rospy.loginfo("Cancelling move_base goal")


    def set_mode(self, mode):
        now = rospy.Time.now()
        if now - self.last_set_mode_time < rospy.Duration(1.0):
            return
        self.last_set_mode_time = now
        self.set_twist_zero()
        rospy.loginfo("Set robot mode to %s. %s" % (mode, self.set_robot_mode(mode)))

    def set_speed_mode(self, value):
        self.speed_mode = value
        if self.speed_mode < 0:
            self.speed_mode = 0
        elif self.speed_mode >= len(self.speed_multipliers):
            self.speed_mode = len(self.speed_multipliers) - 1
        rospy.loginfo("Set speed mode to %s" % self.speed_mode)
        multiplier = self.speed_multipliers[self.speed_mode]
        self.linear_x_scale = multiplier * self.linear_x_scale_max
        self.linear_y_scale = multiplier * self.linear_y_scale_max
        self.angular_scale = multiplier * self.angular_scale_max

    def set_twist(self, linear_x_val, linear_y_val, angular_val):
        if (self.twist_command.linear.x != linear_x_val or 
                self.twist_command.linear.y != linear_y_val or 
                self.twist_command.angular.z != angular_val):
            self.twist_command.linear.x = linear_x_val
            self.twist_command.linear.y = linear_y_val
            self.twist_command.angular.z = angular_val
    
    def set_twist_zero(self):
        self.twist_command.linear.x = 0.0
        self.twist_command.linear.y = 0.0
        self.twist_command.angular.z = 0.0

    def set_field_relative(self, is_field_relative):
        msg = Bool()
        msg.data = is_field_relative
        self.is_field_relative = is_field_relative
        rospy.loginfo("Setting field relative to %s" % is_field_relative)
        self.set_field_relative_pub.publish(msg)

    def run(self):
        # did_disable_timer_expire = False
        # while not rospy.is_shutdown():
        #     # rospy.loginfo(rospy.Time.now() - self.disable_timer, self.disable_timeout, did_disable_timer_expire)
        #     if rospy.Time.now() - self.disable_timer > self.disable_timeout:
        #         if not did_disable_timer_expire:
        #             rospy.loginfo(self.set_robot_mode(RobotStatus.DISABLED))
        #             did_disable_timer_expire = True
        #     else:
        #         did_disable_timer_expire = False     
        #     rospy.sleep(0.1)

        clock_rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            dt = rospy.Time.now() - self.cmd_vel_timer
            if dt > self.cmd_vel_timeout:
                self.set_twist_zero()
            if dt < self.send_timeout:
                self.cmd_vel_pub.publish(self.twist_command)
            clock_rate.sleep()

        # rospy.spin()


if __name__ == "__main__":
    try:
        node = TJ2DebugJoystick()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting tj2_debug_joystick node")
