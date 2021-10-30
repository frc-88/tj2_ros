#!/usr/bin/python3
import rospy
import math

import tf2_ros
import tf_conversions

from std_msgs.msg import Float64
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

from tj2_tools.robot_state import Pose2d
from tj2_tools.tunnel.server import TunnelServer


class TJ2ServerTest(TunnelServer):
    def __init__(self):
        self.categories = {
            "ping": "f",
            "cmd": "fff"
        }

        self.node_name = "server_test"
        rospy.init_node(
            self.node_name,
            # disable_signals=True,
            log_level=rospy.DEBUG
        )

        self.host = rospy.get_param("~host", "")
        self.port = rospy.get_param("~port", 3000)

        self.clock_rate = rospy.Rate(50.0)
        
        super(TJ2ServerTest, self).__init__(self.host, self.port, self.categories)
        rospy.loginfo("%s init complete" % self.node_name)

    def packet_callback(self, error_code, recv_time, category, data):
        if category == "ping":
            self.write("ping", data[0])

        elif category == "cmd":
            rospy.loginfo("Command. v=%0.4f, t=%0.4f, vt=%0.4f" % tuple(data))

    def run(self):
        try:
            self.start()
            while not rospy.is_shutdown():
                self.clock_rate.sleep()
                self.update()
                self.write("odom", 0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
        finally:
            self.stop()


if __name__ == "__main__":
    node = TJ2ServerTest()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
