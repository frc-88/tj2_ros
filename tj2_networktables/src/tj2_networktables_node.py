#!/usr/bin/python
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from networktables import NetworkTables


class TJ2NetworkTables(object):
    def __init__(self):
        self.node_name = "tj2_networktables"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        NetworkTables.initialize(server="10.0.88.2")
        self.nt = NetworkTables.getTable("Swerve")

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"

        self.inch_to_m = 0.0254

        self.remote_start_time = None
        self.local_start_time = None
        self.prev_timestamp = 0.0
        
        self.clock_rate = rospy.Rate(50.0)


    def run(self):
        while not rospy.is_shutdown():
            self.clock_rate.sleep()
            timestamp = self.nt.getEntry("odom/time").getDouble(0.0)
            self.nt.putNumber("command/time", rospy.Time.now().to_sec())
            print timestamp



if __name__ == "__main__":
    node = TJ2NetworkTables()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
