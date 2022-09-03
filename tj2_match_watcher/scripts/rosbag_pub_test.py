#!/usr/bin/python3
import rospy

from std_msgs.msg import Int32

from std_srvs.srv import Empty
from tj2_match_watcher.srv import TriggerBag


class RosbagPubTest:
    def __init__(self):
        self.name = "rosbag_pub_test"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.publisher = rospy.Publisher("counter", Int32, queue_size=10)

        self.start_bag_srv = self.make_service_client("/rosbag_controlled_recording/start", TriggerBag, wait=True, timeout=30.0)
        self.resume_bag_srv = self.make_service_client("/rosbag_controlled_recording/resume", Empty, wait=True, timeout=30.0)
        self.pause_bag_srv = self.make_service_client("/rosbag_controlled_recording/pause", Empty, wait=True, timeout=30.0)
        self.stop_bag_srv = self.make_service_client("/rosbag_controlled_recording/stop", TriggerBag, wait=True, timeout=30.0)

        rospy.loginfo("%s init complete" % self.name)
    
    def make_service_client(self, name, srv_type, timeout=None, wait=True):
        """
        Create a ros service client. Optionally wait with or without a timeout for the server to connect
        """
        self.__dict__[name + "_service_name"] = name
        rospy.loginfo("Connecting to %s service" % name)
        srv_obj = rospy.ServiceProxy(name, srv_type)

        if wait:
            rospy.loginfo("Waiting for service %s" % name)
            rospy.wait_for_service(name, timeout=timeout)
            rospy.loginfo("%s service is ready" % name)
        return srv_obj
    
    def run(self):
        counter = 0

        print(self.start_bag_srv())

        while not rospy.is_shutdown():
            msg = Int32()
            msg.data = counter
            self.publisher.publish(msg)
            counter += 1
            rospy.sleep(1.0)


if __name__ == "__main__":
    node = RosbagPubTest()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
