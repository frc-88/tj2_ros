#!/usr/bin/python3
import os
import rospy
import rospkg

from std_msgs.msg import Time

from tj2_tools.launch_manager import LaunchManager

PREGAME = -1
AUTONOMOUS = 0
TELEOP = 1
ENDGAME = 2
FINISHED = 3
TIMEOUT = 4

PERIODS = {
    PREGAME: "PREGAME",
    AUTONOMOUS: "AUTONOMOUS",
    TIMEOUT: "TIMEOUT",
    TELEOP: "TELEOP",
    ENDGAME: "ENDGAME",
    FINISHED: "FINISHED",
}



class TJ2MatchWatcher(object):
    def __init__(self):
        self.node_name = "tj2_match_watcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
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
    
    def make_service(self, name, srv_type, callback):
        rospy.loginfo("Setting up service %s" % name)
        srv_obj = rospy.Service(name, srv_type, callback)
        rospy.loginfo("%s service is ready" % name)
        return srv_obj

    def start_bag_callback(self, msg: Time):
        rospy.loginfo("Resuming match bag")
        self.resume_bag()
    
    def start_bag(self):
        rospy.loginfo("Starting match bag")

    def resume_bag(self):
        rospy.loginfo("Resuming match bag")
        self.bag_launcher.start()
    
    def stop_bag(self):
        rospy.loginfo("Stopping match bag")
        self.bag_launcher.stop()
    
    def run(self):
        self.start_bag()
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
