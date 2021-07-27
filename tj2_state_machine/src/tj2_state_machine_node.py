#!/usr/bin/python3
import rospy

from std_srvs.srv import Trigger
from std_msgs.msg import Float64

NOPERIOD = -1
AUTONOMOUS = 0
TELEOP = 1
ENDGAME = 2
FINISHED = 3

PERIODS = {
    NOPERIOD: "NOPERIOD",
    AUTONOMOUS: "AUTONOMOUS",
    TELEOP: "TELEOP",
    ENDGAME: "ENDGAME",
    FINISHED: "FINISHED",
}



class TJ2StateMachine(object):
    def __init__(self):
        self.node_name = "tj2_state_machine"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.loginfo("%s init complete" % self.node_name)

        self.autonomous_period_s = 15.0
        self.teleop_period_s = 105.0 + self.autonomous_period_s
        self.end_game_period_s = 30.0 + self.teleop_period_s

        self.period = NOPERIOD

        self.start_camera_srv = self.make_service_client("/tj2/start_camera", Trigger, wait=True)
        self.stop_camera_srv = self.make_service_client("/tj2/stop_camera", Trigger, wait=False)
        self.start_record_srv = self.make_service_client("/tj2/start_record", Trigger, wait=False)
        self.stop_record_srv = self.make_service_client("/tj2/stop_record", Trigger, wait=False)

        response = self.start_camera_srv()
        rospy.loginfo(str(response))

        self.match_time_sub = rospy.Subscriber("match_time", Float64, self.match_time_callback, queue_size=100)

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
    
    def match_time_callback(self, msg):
        if msg.data < 0.0:
            period = NOPERIOD
        elif 0.0 <= msg.data < self.autonomous_period_s:
            period = AUTONOMOUS
        elif self.autonomous_period_s <= msg.data < self.teleop_period_s:
            period = TELEOP
        elif self.teleop_period_s <= msg.data < self.end_game_period_s:
            period = ENDGAME
        else:
            period = FINISHED
        if period != self.period:
            self.period = period
            self.period_changed(self.period)

    def period_changed(self, period):
        rospy.loginfo("Game is now in the %s period" % PERIODS.get(period, NOPERIOD).lower())
        if period == ENDGAME:
            response = self.start_record_srv()
            rospy.loginfo(str(response))
        
        if period != NOPERIOD and period != FINISHED:
            response = self.start_camera_srv()
            rospy.loginfo(str(response))

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
