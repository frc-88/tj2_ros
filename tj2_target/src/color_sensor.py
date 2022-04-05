#!/usr/bin/python3
import rospy

from std_msgs.msg import ColorRGBA
from std_msgs.msg import Int16

from tj2_networktables.msg import NTEntry

from tj2_tools.rev_color_sensor import ColorSensorV3


class ColorSensorNode:
    def __init__(self):
        self.name = "color_sensor"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.nt_pub = rospy.Publisher("nt_passthrough", NTEntry, queue_size=10)
        self.color_pub = rospy.Publisher("color_sensor", ColorRGBA, queue_size=10)
        self.proximity_pub = rospy.Publisher("color_proximity", Int16, queue_size=10)

        self.color_sensor = ColorSensorV3()

        rospy.loginfo("%s init complete" % self.name)

    def run(self):
        clock = rospy.Rate(10.0)
        
        while not rospy.is_shutdown():
            color = self.color_sensor.getColor()
            color_msg = ColorRGBA(color.red, color.green, color.blue, 1.0)
            self.color_pub.publish(color_msg)

            proximity = self.color_sensor.getProximity()
            proximity_msg = Int16(proximity)
            self.proximity_pub.publish(proximity_msg)

            self.nt_pub.publish(self.make_entry("color_sensor/r", color.red))
            self.nt_pub.publish(self.make_entry("color_sensor/g", color.green))
            self.nt_pub.publish(self.make_entry("color_sensor/b", color.blue))
            self.nt_pub.publish(self.make_entry("color_sensor/proximity", proximity))

            clock.sleep()


if __name__ == "__main__":
    node = ColorSensorNode()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.name)
