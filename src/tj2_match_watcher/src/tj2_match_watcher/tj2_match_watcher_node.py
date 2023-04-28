#!/usr/bin/python3
import os
import rospy
import rospkg

from std_msgs.msg import Time

from tj2_tools.launch_manager import LaunchManager


class TJ2MatchWatcher(object):
    def __init__(self):
        self.node_name = "tj2_match_watcher"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.shutdown_hook)

        self.record_match_launch_path = rospy.get_param("~record_match_launch_path", "")
        if len(self.record_match_launch_path) == 0:
            self.rospack = rospkg.RosPack()
            self.package_dir = self.rospack.get_path(self.node_name)
            self.default_launches_dir = self.package_dir + "/launch"
            self.record_match_launch_path = self.default_launches_dir + "/record_match.launch"
        if not os.path.isfile(self.record_match_launch_path):
            raise RuntimeError("Record launch path does not exist: %s" % self.record_match_launch_path)

        self.bag_launcher = LaunchManager(self.record_match_launch_path)

        self.start_recording_sub = rospy.Subscriber("/tj2/start_bag", Time, self.start_bag_callback, queue_size=10)

        rospy.loginfo("%s init complete" % self.node_name)

    def start_bag_callback(self, msg: Time):
        rospy.loginfo("Resuming match bag")
        # self.start_bag()
    
    def start_bag(self):
        rospy.loginfo("Resuming match bag")
        self.bag_launcher.start()
    
    def stop_bag(self):
        rospy.loginfo("Stopping match bag")
        self.bag_launcher.stop()
    
    def run(self):
        rospy.spin()

    def shutdown_hook(self):
        rospy.loginfo("Shutdown called. Stopping bag")
        self.stop_bag()


if __name__ == "__main__":
    node = TJ2MatchWatcher()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
