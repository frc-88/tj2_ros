#!/usr/bin/env python3
import rospy
import rostopic

from dynamic_reconfigure.client import Client as DynamicClient


class TJ2LidarWatcher:
    def __init__(self):
        self.node_name = "tj2_lidar_watcher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.expected_sinister_rate = rospy.get_param("~expected_sinister_rate", 10.0)
        self.expected_dexter_rate = rospy.get_param("~expected_dexter_rate", 10.0)
        self.min_rate_offset = rospy.get_param("~rate_band", 5.0)
        self.sinister_min_rate_threshold = max(0.0, self.expected_sinister_rate - self.min_rate_offset)
        self.sinister_max_rate_threshold = max(0.0, self.expected_sinister_rate + self.min_rate_offset)
        self.dexter_min_rate_threshold = max(0.0, self.expected_dexter_rate - self.min_rate_offset)
        self.dexter_max_rate_threshold = max(0.0, self.expected_dexter_rate + self.min_rate_offset)
        self.combiner_config = rospy.get_param("~combiner_config", None)

        self.sinister_rate = rostopic.ROSTopicHz(15)
        self.sinister_topic = "/sinister_laser/scan_filtered"
        rospy.Subscriber(self.sinister_topic, rospy.AnyMsg, self.sinister_rate.callback_hz, callback_args=self.sinister_topic, queue_size=5)

        self.dexter_rate = rostopic.ROSTopicHz(15)
        self.dexter_topic = "/dexter_laser/scan_filtered"
        rospy.Subscriber(self.dexter_topic, rospy.AnyMsg, self.dexter_rate.callback_hz, callback_args=self.dexter_topic, queue_size=5)

        self.combiner_client = None
        self.combiner_dyn_topic = "/laserscan_multi_merger"

        rospy.loginfo("%s init complete" % self.node_name)

    def init_dynamic_clients(self):
        if self.combiner_client is None:
            self.combiner_client = DynamicClient(self.combiner_dyn_topic)

    def get_publish_rate(self):
        sinister_result = self.sinister_rate.get_hz(self.sinister_topic)
        dexter_result = self.dexter_rate.get_hz(self.dexter_topic)
        sinister_rate = 0.0 if sinister_result is None else sinister_result[0]
        dexter_rate = 0.0 if dexter_result is None else dexter_result[0]
        return sinister_rate, dexter_rate

    def set_parameters(self):
        self.init_dynamic_clients()
        if self.combiner_config is not None:
            rospy.loginfo("Updating laser combiner parameters: %s" % str(self.combiner_config))
            rospy.wait_for_service(self.combiner_dyn_topic + "/set_parameters", 30.0)
            self.combiner_client.update_configuration(self.combiner_config)
        else:
            rospy.loginfo("laser combiner parameters are set. Skipping dynamic reconfigure")

    def run(self):
        parameters_set = False
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            sinister_rate, dexter_rate = self.get_publish_rate()
            if not parameters_set and (sinister_rate > 0.0 or dexter_rate > 0.0):
                self.set_parameters()
                parameters_set = True

            if not (self.sinister_min_rate_threshold <= sinister_rate <= self.sinister_max_rate_threshold and
                    self.dexter_min_rate_threshold <= dexter_rate <= self.dexter_max_rate_threshold):
                rospy.logwarn_throttle(2.0, 
                    "LIDARs are not publishing within the threshold. " \
                    "sinister (%0.1f..%0.1f): %0.2f. dexter (%0.1f..%0.1f): %0.2f" % (
                        self.sinister_min_rate_threshold, self.sinister_max_rate_threshold, sinister_rate,
                        self.dexter_min_rate_threshold, self.dexter_max_rate_threshold, dexter_rate)
                )


if __name__ == "__main__":
    node = TJ2LidarWatcher()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
