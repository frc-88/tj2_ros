#!/usr/bin/env python3
import os
from enum import IntEnum

import rospy
from std_msgs.msg import Int32
from tj2_tools.launch_manager import LaunchManager


class JetsonPowerMode(IntEnum):
    UNKNOWN = -1
    LOW_POWER = 3
    HIGH_POWER = 6


class PowerModeNode:
    def __init__(self) -> None:
        self.default_mode_id = JetsonPowerMode(rospy.get_param("~default_power_mode", JetsonPowerMode.HIGH_POWER.value))
        self.launch_trigger = JetsonPowerMode(rospy.get_param("~launch_trigger", JetsonPowerMode.HIGH_POWER.value))
        self.launch_path = str(rospy.get_param("~launch_path", ""))
        self.primary_launcher = LaunchManager(self.launch_path) if self.launch_path else None
        self.current_mode_id = JetsonPowerMode.UNKNOWN
        self.prev_mode_set_time = rospy.Time(0)
        self.mode_set_cooldown = rospy.Duration.from_sec(3.0)
        self.mode_sub = rospy.Subscriber("power_mode", Int32, self.mode_callback, queue_size=1)

    def mode_callback(self, msg: Int32) -> None:
        self.set_power_mode(JetsonPowerMode(msg.data))

    def set_power_mode(self, mode_id: JetsonPowerMode) -> None:
        if self.current_mode_id == mode_id:
            return
        if mode_id == JetsonPowerMode.UNKNOWN:
            rospy.logwarn("Cannot set power mode to UNKNOWN")
            return
        if rospy.Time.now() - self.prev_mode_set_time < self.mode_set_cooldown:
            rospy.logwarn(
                f"Power mode set too recently. "
                f"Please wait {self.mode_set_cooldown.to_sec()} seconds. "
                "Ignoring request."
            )
            return

        self.current_mode_id = mode_id
        self.prev_mode_set_time = rospy.Time.now()

        rospy.loginfo(f"Setting power mode to {mode_id}")
        if self.primary_launcher:
            if self.launch_trigger == mode_id:
                rospy.loginfo("Starting primary launcher")
                self.primary_launcher.start()
            else:
                rospy.loginfo("Stopping primary launcher")
                self.primary_launcher.stop()
        os.system(f"sudo /usr/sbin/nvpmodel -m {mode_id.value}")

    def run(self):
        self.set_power_mode(self.default_mode_id)
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("power_mode_node")
    PowerModeNode().run()
