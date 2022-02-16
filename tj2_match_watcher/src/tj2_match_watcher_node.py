#!/usr/bin/python3
import rospy

from std_srvs.srv import Empty
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from tj2_match_watcher.srv import TriggerBag

PREGAME = -1
AUTONOMOUS = 0
TELEOP = 1
ENDGAME = 2
FINISHED = 3

PERIODS = {
    PREGAME: "PREGAME",
    AUTONOMOUS: "AUTONOMOUS",
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
        rospy.loginfo("%s init complete" % self.node_name)

        self.autonomous_period_s = 15.0
        self.teleop_period_s = 135.0
        self.end_game_period_s = 30.0

        self.is_autonomous = True
        self.period = PREGAME
        self.prev_match_time = 0.0

        self.start_bag_srv = self.make_service_client("/rosbag_controlled_recording/start", TriggerBag, wait=True)
        self.resume_bag_srv = self.make_service_client("/rosbag_controlled_recording/resume", Empty, wait=True)
        self.pause_bag_srv = self.make_service_client("/rosbag_controlled_recording/pause", Empty, wait=True)
        self.stop_bag_srv = self.make_service_client("/rosbag_controlled_recording/stop", TriggerBag, wait=True)
        self.bag_is_started = False
        self.bag_name = ""

        self.match_time_sub = rospy.Subscriber("match_time", Float64, self.match_time_callback, queue_size=10)
        self.is_autonomous_sub = rospy.Subscriber("is_autonomous", Bool, self.is_autonomous_callback, queue_size=10)

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
        match_time = msg.data
        if self.prev_match_time == match_time:
            return 
        rospy.loginfo_throttle(1, "Match time: %s" % match_time)
        self.prev_match_time = match_time
        
        if match_time <= 0.0:
            period = PREGAME
        else:
            if self.is_autonomous:
                period = AUTONOMOUS
                if not (0.0 <= match_time < self.autonomous_period_s):
                    rospy.logwarn_throttle(15, "is_autonomous is true but the match time %s doesn't line up!" % match_time)
            else:
                if self.end_game_period_s <= msg.data < self.teleop_period_s:
                    period = TELEOP
                elif msg.data < self.end_game_period_s:
                     period = ENDGAME
                else:
                    period = FINISHED
        if period != self.period:
            self.period = period
            self.period_changed(self.period)

    def period_changed(self, period):
        rospy.loginfo("Game is now in the %s period" % PERIODS.get(period, PREGAME).lower())
        if period == AUTONOMOUS:
            self.start_bag()
            self.resume_bag_srv()
        elif period == FINISHED:
            self.stop_bag()

    def start_bag(self):
        if self.bag_is_started:
            return
        response = self.start_bag_srv()
        if not response.success:
            raise RuntimeError("Failed to create bag!")
        self.bag_name = response.bag_name
        rospy.loginfo("Starting bag: %s" % self.bag_name)
        self.bag_is_started = True
        rospy.sleep(5.0)  # record 5 seconds to ensure bag is running before pausing
        self.pause_bag_srv()
    
    def stop_bag(self):
        self.stop_bag_srv()
        self.bag_is_started = False

    def run(self):
        self.start_bag()
        rospy.spin()


if __name__ == "__main__":
    node = Tj2MatchWatcher()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
