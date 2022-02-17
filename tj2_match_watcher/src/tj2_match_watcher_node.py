#!/usr/bin/python3
import rospy

from std_srvs.srv import Empty, EmptyResponse
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
        self.full_match_s = self.autonomous_period_s + self.teleop_period_s + self.end_game_period_s

        self.match_definitely_over_duration = rospy.Duration(self.full_match_s + 60.0)

        self.game_start_time = rospy.Time(0)

        self.is_autonomous = True
        self.period = PREGAME
        self.prev_match_time = 0.0

        self.bag_is_started = True  # bag is started by default in the paused state
        self.bag_name = ""

        self.start_bag_srv = self.make_service_client("/rosbag_controlled_recording/start", TriggerBag, wait=False)
        self.resume_bag_srv = self.make_service_client("/rosbag_controlled_recording/resume", Empty, wait=False)
        self.pause_bag_srv = self.make_service_client("/rosbag_controlled_recording/pause", Empty, wait=False)
        self.stop_bag_srv = self.make_service_client("/rosbag_controlled_recording/stop", TriggerBag, wait=False)

        self.reset_srv = self.make_service("reset_match", Empty, self.reset_match)

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
        self.prev_match_time = match_time

        if match_time <= 0.0:
            if self.game_start_time > rospy.Time(0):
                period = FINISHED
            else:
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
        if self.game_start_time > rospy.Time(0):
            match_duration = rospy.Time.now() - self.game_start_time
            if match_duration > self.match_definitely_over_duration:
                rospy.loginfo("Match is definitely over based on time limit")
                if self.bag_is_started:
                    self.stop_bag()
                self.reset_match()

        rospy.loginfo_throttle(1, "Match time [%s]: %0.1f s" % (PERIODS.get(period, PREGAME).lower(), match_time))
        if period != self.period:
            if period == AUTONOMOUS:
                self.game_start_time = rospy.Time.now()
                rospy.loginfo("Game started at %0.1f" % self.game_start_time.to_sec())
            self.period = period
            self.period_changed(self.period)

    def reset_match(self, req=None):
        rospy.loginfo("Resetting match")
        self.game_start_time = rospy.Time(0)

        self.is_autonomous = True
        self.period = PREGAME
        self.prev_match_time = 0.0

        self.start_bag()
        self.bag_is_started = True  # bag is started by default in the paused state
        self.bag_name = ""

        return EmptyResponse()

    def period_changed(self, period):
        rospy.loginfo("Game is now in the %s period" % PERIODS.get(period, PREGAME).lower())
        if period == AUTONOMOUS:
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

    def stop_bag(self):
        if not self.bag_is_started:
            return
        rospy.loginfo("Bag %s is stopping" % self.bag_name)
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
