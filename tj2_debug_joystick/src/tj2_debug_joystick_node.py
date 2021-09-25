#!/usr/bin/env python3
import traceback
import math

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from tj2_tools.joystick import Joystick
from tj2_driver_station.srv import SetRobotMode
from tj2_driver_station.msg import RobotStatus


class TJ2DebugJoystick:
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

        self.cmd_vel_timeout = rospy.Time.now()

        # parameters from launch file
        self.linear_x_axis = rospy.get_param("~linear_x_axis", "left/X").split("/")
        self.linear_y_axis = rospy.get_param("~linear_y_axis", "left/Y").split("/")
        self.angular_axis = rospy.get_param("~angular_axis", "right/X").split("/")

        self.linear_x_scale = rospy.get_param("~linear_x_scale", 1.0)
        self.linear_y_scale = rospy.get_param("~linear_y_scale", 1.0)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)

        self.deadzone_joy_val = rospy.get_param("~deadzone_joy_val", 0.05)
        self.joystick_topic = rospy.get_param("~joystick_topic", "/joy")

        self.button_mapping = rospy.get_param("~button_mapping", None)
        assert self.button_mapping is not None
        self.axis_mapping = rospy.get_param("~axis_mapping", None)
        assert self.axis_mapping is not None

        # services
        self.set_robot_mode = rospy.ServiceProxy("robot_mode", SetRobotMode)

        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)

        self.joystick = Joystick(self.button_mapping, self.axis_mapping)

        # subscription topics
        self.joy_sub = rospy.Subscriber(self.joystick_topic, Joy, self.joystick_msg_callback, queue_size=5)

        rospy.loginfo("Debug joystick is ready!")

    def joystick_msg_callback(self, msg):
        self.joystick.update(msg)
        
        if self.joystick.did_button_down(("triggers", "L1")):
            rospy.loginfo(self.set_robot_mode(RobotStatus.TELEOP))
        elif self.joystick.did_button_down(("triggers", "R1")):
            rospy.loginfo(self.set_robot_mode(RobotStatus.DISABLED))

        if any(self.joystick.check_list(self.joystick.did_axis_change, self.linear_x_axis, self.linear_y_axis, self.angular_axis)):
            linear_x_val = self.joystick.deadband_axis(self.linear_x_axis, self.deadzone_joy_val, self.linear_x_scale)
            linear_y_val = self.joystick.deadband_axis(self.linear_y_axis, self.deadzone_joy_val, self.linear_y_scale)
            angular_val = self.joystick.deadband_axis(self.angular_axis, self.deadzone_joy_val, self.angular_scale)

            if self.set_twist(linear_x_val, linear_y_val, angular_val):
                self.cmd_vel_pub.publish(self.twist_command)


    def set_twist(self, linear_x_val, linear_y_val, angular_val):
        if (self.twist_command.linear.x != linear_x_val or 
                self.twist_command.linear.y != linear_y_val or 
                self.twist_command.angular.z != angular_val):
            self.twist_command.linear.x = linear_x_val
            self.twist_command.linear.y = linear_y_val
            self.twist_command.angular.z = angular_val
            self.cmd_vel_timeout = rospy.Time.now()
            return True
        else:
            return rospy.Time.now() - self.cmd_vel_timeout < rospy.Duration(0.5)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = TJ2DebugJoystick()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting tj2_debug_joystick node")
