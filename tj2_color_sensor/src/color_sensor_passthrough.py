#!/usr/bin/env python3
import rospy

from rev_color_sensor_ros.msg import RevColorSensor

from tj2_interfaces.msg import NTEntry

class ColorSensorPassthrough:
    def __init__(self):
        self.node_name = "color_sensor_passthrough"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.namespace = rospy.get_param("~namespace", "")

        sensor_topic = "color_sensor/sensor"
        if len(self.namespace) != 0:
            topic = self.namespace + sensor_topic
        else:
            topic = sensor_topic

        self.color_sensor_sub = rospy.Subscriber(topic, RevColorSensor, self.color_sensor_callback, queue_size=10)
        self.nt_passthrough_pub = rospy.Publisher("nt_passthrough", NTEntry, queue_size=10)
    
    def color_sensor_callback(self, msg):
        if len(self.namespace) == 0:
            parent_table = "color_sensor"
        else:
            parent_table = "color_sensor/" + self.namespace

        match_entry = NTEntry()
        match_entry.path = parent_table + "/match"
        match_entry.value = msg.match

        self.nt_passthrough_pub.publish(match_entry)
    
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = ColorSensorPassthrough()
    node.run()
