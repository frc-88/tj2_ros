#!/usr/bin/python3
import os
import rospy
import rospkg

from std_msgs.msg import Float64
from std_msgs.msg import Bool

from geometry_msgs.msg import PoseWithCovarianceStamped

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



class Tj2MatchWatcher(object):
    def __init__(self):
        self.node_name = "tj2_match_watcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.autonomous_period_s = 15.0
        self.timeout_period_s = 1.5
        self.teleop_period_s = 135.0
        self.end_game_period_s = 30.0
        self.full_match_s = self.autonomous_period_s + self.teleop_period_s + self.timeout_period_s

        self.bag_time_buffer = 90.0

        self.match_definitely_over_duration = rospy.Duration(self.full_match_s + self.bag_time_buffer)

        self.game_start_time = rospy.Time(0)

        self.match_time = -1.0
        self.is_autonomous = False

        self.record_match_launch_path = rospy.get_param("~record_match_launch_path", "")
        if len(self.record_match_launch_path) == 0:
            self.rospack = rospkg.RosPack()
            self.package_dir = self.rospack.get_path(self.node_name)
            self.default_launches_dir = self.package_dir + "/launch"
            self.record_match_launch_path = self.default_launches_dir + "/record_match.launch"
        if not os.path.isfile(self.record_match_launch_path):
            raise RuntimeError("Record launch path does not exist: %s" % self.record_match_launch_path)

        self.bag_launcher = LaunchManager(self.record_match_launch_path)

        self.match_time_sub = rospy.Subscriber("match_time", Float64, self.match_time_callback, queue_size=10)
        self.is_autonomous_sub = rospy.Subscriber("is_autonomous", Bool, self.is_autonomous_callback, queue_size=10)
        self.initial_pose_sub = rospy.Subscriber("/tj2/reset_pose", PoseWithCovarianceStamped, self.initialpose_callback, queue_size=10)

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

    def is_autonomous_callback(self, msg):
        self.is_autonomous = msg.data
    
    def match_time_callback(self, msg):
        self.match_time = msg.data
    
    def initialpose_callback(self, msg):
        rospy.loginfo("Initial pose set! Resuming match bag")
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
        self.is_autonomous = False
        clock = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_autonomous and (0.0 < self.match_time < 14.0):
                if self.game_start_time <= rospy.Time(0):
                    self.game_start_time = rospy.Time.now()
                    rospy.loginfo("Detected autonomous mode. Starting recording")
                    self.resume_bag()

            if self.game_start_time > rospy.Time(0):
                match_duration = rospy.Time.now() - self.game_start_time
                rospy.loginfo_throttle(0.25, "Match timer: %s. Match start time: %s" % (match_duration.to_sec(), self.game_start_time.to_sec()))
                if match_duration > self.match_definitely_over_duration:
                    rospy.loginfo("End of the match timer expired. Stopping bag")
                    self.stop_bag()
                    self.game_start_time = rospy.Time(0)
            clock.sleep()

    def shutdown_hook(self):
        rospy.loginfo("Shutdown called. Stopping bag")
        self.stop_bag()


if __name__ == "__main__":
    node = Tj2MatchWatcher()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
