#!/usr/bin/python3
import rospy


class TJ2StateMachine(object):
    def __init__(self):
        self.node_name = "tj2_state_machine"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = TJ2StateMachine()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
