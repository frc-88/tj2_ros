#!/usr/bin/env python3
import traceback
import math

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class TJ2DebugJoystick:
    def __init__(self):
        rospy.init_node(
            "tj2_debug_joystick",
            disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.prev_joy_msg = None

        self.max_joy_val = 1.0

        self.twist_command = Twist()
        self.twist_command.linear.x = 0.0
        self.twist_command.linear.y = 0.0
        self.twist_command.angular.z = 0.0

        self.cmd_vel_timeout = rospy.Time.now()

        # parameters from launch file
        self.linear_x_axis = int(rospy.get_param("~linear_x_axis", 1))
        self.linear_y_axis = int(rospy.get_param("~linear_y_axis", 0))
        self.angular_axis = int(rospy.get_param("~angular_axis", 2))

        self.linear_x_scale = rospy.get_param("~linear_x_scale", 1.0)
        self.linear_y_scale = rospy.get_param("~linear_y_scale", 1.0)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)

        self.deadzone_joy_val = rospy.get_param("~deadzone_joy_val", 0.05)
        self.joystick_topic = rospy.get_param("~joystick_topic", "/joy")

        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)

        # subscription topics
        self.joy_sub = rospy.Subscriber(self.joystick_topic, Joy, self.joystick_msg_callback, queue_size=5)

        rospy.loginfo("Debug joystick is ready!")

    def joy_to_speed(self, scale_factor, value):
        if abs(value) < self.deadzone_joy_val:
            return 0.0
        joy_val = abs(value) - self.deadzone_joy_val
        joy_val = math.copysign(joy_val, value)
        max_joy_val_adj = self.max_joy_val - self.deadzone_joy_val
        command = scale_factor / max_joy_val_adj * joy_val

        return command

    def is_button_down(self, msg, index):
        return msg.buttons[index] and self.did_button_change(msg, index)

    def is_button_up(self, msg, index):
        return not msg.buttons[index] and self.did_button_change(msg, index)

    def did_button_change(self, msg, index):
        return self.prev_joy_msg.buttons[index] != msg.buttons[index]

    def did_axis_change(self, msg, index):
        return self.prev_joy_msg.axes[index] != msg.axes[index]

    def joystick_msg_callback(self, msg):
        try:
            self.process_joy_msg(msg)
        except BaseException as e:
            traceback.print_exc()
            rospy.signal_shutdown(str(e))

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

    def process_joy_msg(self, msg):
        if self.prev_joy_msg is None:
            self.prev_joy_msg = msg
            return

        # Xbox button mapping:
        # 0: A,    1: B,     2: X,      3: Y
        # 4: L1,   5: R1,    6: Select, 7: Start
        # 8: L joy, 9: R joy

        # Xbox axes:
        # Lx: 0, Ly: 1
        # Rx: 2, Ry: 3
        # L brake: 5
        # R brake: 4
        # D-pad left-right: 6
        # D-pad up-down: 7
        # if self.is_button_down(msg, 0):  # A
        # if self.is_button_down(msg, 1): # B
        # if self.is_button_down(msg, 3): # Y
        # if self.is_button_down(msg, 2): # X
        # if self.did_button_change(msg, 4): # L1

        # if self.did_axis_change(msg, 6):
        #     if msg.axes[6] > 0:  # D-pad left
        #     elif msg.axes[6] < 0:  # D-pad right
        # if self.did_axis_change(msg, 7):
        #     if msg.axes[7] > 0:  # D-pad up
        #     elif msg.axes[7] < 0:  # D-pad down

        linear_x_val = self.joy_to_speed(self.linear_x_scale, msg.axes[self.linear_x_axis])
        linear_y_val = self.joy_to_speed(self.linear_y_scale, msg.axes[self.linear_y_axis])
        angular_val = self.joy_to_speed(self.angular_scale, msg.axes[self.angular_axis])

        if self.set_twist(linear_x_val, linear_y_val, angular_val):
            self.cmd_vel_pub.publish(self.twist_command)

        self.prev_joy_msg = msg

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = TJ2DebugJoystick()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting tj2_debug_joystick node")
