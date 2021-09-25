#!/usr/bin/python3
import math
import rospy

from dynamic_reconfigure.server import Server
from tj2_networktables.cfg import SwerveConstants

from networktables import NetworkTables


class ConstantsRelay:
    def __init__(self):
        self.node_name = "constants_relay"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.nt_host = rospy.get_param("~nt_host", "10.0.88.2")
        
        NetworkTables.initialize(server=self.nt_host)
        self.nt = NetworkTables.getTable("swerveLibrary")

        # self.nt.addEntryListener(self.ping_callback, key="pingResponse")

        self.dyn_srv = Server(SwerveConstants, self.swerve_constants_callback)

        rospy.loginfo("%s init complete" % self.node_name)

    def swerve_constants_callback(self, config, level):
        rospy.loginfo("Level: %s" % level)
        rospy.loginfo(
            "Reconfigure Request: {int_param}, {double_param}, " \
            "{str_param}, {bool_param}, {size}".format(**config)
        )
        return config

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = ConstantsRelay()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
