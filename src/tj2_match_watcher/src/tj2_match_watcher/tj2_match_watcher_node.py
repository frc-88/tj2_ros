#!/usr/bin/python3
import datetime

import rospy
from std_msgs.msg import Time
from tj2_interfaces.msg import SystemSummary
from tj2_match_watcher.record_bag_manager import RecordBagManager
from tj2_match_watcher.svo_service_manager import SvoServiceManager
from tj2_tools.environment import get_robot, get_system_info


class TJ2MatchWatcher(object):
    def __init__(self):
        self.node_name = "tj2_match_watcher"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.shutdown_hook)

        exclude_regex = rospy.get_param("~exclude_regex", "")
        self.svo_manager = SvoServiceManager("/tj2_zed", "/media/storage/svo")
        self.bag_manager = RecordBagManager("/media/storage/bags", exclude_regex=exclude_regex)

        self.start_bag_sub = rospy.Subscriber("start_bag", Time, self.start_bag_callback, queue_size=1)
        self.start_svo_sub = rospy.Subscriber("start_svo", Time, self.start_svo_callback, queue_size=1)
        self.stop_bag_sub = rospy.Subscriber("stop_bag", Time, self.stop_bag_callback, queue_size=1)

        self.summary_pub = rospy.Publisher("summary", SystemSummary, queue_size=1, latch=True)

        self.active_filename = ""

        rospy.loginfo("%s init complete" % self.node_name)

    def get_filename(self):
        now = datetime.datetime.now()
        datestr = now.strftime("%Y-%m-%dT%H-%M-%S")
        return f"{get_robot()}_{datestr}"

    def start_bag_callback(self, msg: Time):
        self.start_bag()

    def start_svo_callback(self, msg: Time):
        self.start_svo()

    def stop_bag_callback(self, msg: Time):
        self.stop_bag()

    def start_bag(self):
        rospy.loginfo("Starting match bag")
        if not self.active_filename:
            self.active_filename = self.get_filename()
        self.bag_manager.start(self.active_filename)

    def start_svo(self):
        rospy.loginfo("Starting svo")
        if not self.active_filename:
            self.active_filename = self.get_filename()
        self.svo_manager.start(self.active_filename)

    def stop_bag(self):
        rospy.loginfo("Stopping match bag")
        self.bag_manager.stop()
        if self.svo_manager.is_recording:
            rospy.loginfo("Stopping svo")
            self.svo_manager.stop()
        self.active_filename = ""

    def run(self):
        self.summary_pub.publish(get_system_info())
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
