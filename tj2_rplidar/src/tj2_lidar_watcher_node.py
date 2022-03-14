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

        self.expected_sinistra_rate = rospy.get_param("~expected_sinistra_rate", 10.0)
        self.expected_dextra_rate = rospy.get_param("~expected_dextra_rate", 10.0)
        self.min_rate_offset = rospy.get_param("~rate_band", 5.0)
        self.sinistra_min_rate_threshold = max(0.0, self.expected_sinistra_rate - self.min_rate_offset)
        self.sinistra_max_rate_threshold = max(0.0, self.expected_sinistra_rate + self.min_rate_offset)
        self.dextra_min_rate_threshold = max(0.0, self.expected_dextra_rate - self.min_rate_offset)
        self.dextra_max_rate_threshold = max(0.0, self.expected_dextra_rate + self.min_rate_offset)
        self.combiner_config = rospy.get_param("~combiner_config", None)

        self.sinistra_rate = rostopic.ROSTopicHz(15)
        self.sinistra_topic = "/sinistra_laser/scan_filtered"
        rospy.Subscriber(self.sinistra_topic, rospy.AnyMsg, self.sinistra_rate.callback_hz, callback_args=self.sinistra_topic, queue_size=5)

        self.dextra_rate = rostopic.ROSTopicHz(15)
        self.dextra_topic = "/dextra_laser/scan_filtered"
        rospy.Subscriber(self.dextra_topic, rospy.AnyMsg, self.dextra_rate.callback_hz, callback_args=self.dextra_topic, queue_size=5)

        self.combiner_client = None
        self.combiner_dyn_topic = "/laserscan_multi_merger"

        rospy.loginfo("%s init complete" % self.node_name)

    def init_dynamic_clients(self):
        if self.combiner_client is None:
            self.combiner_client = DynamicClient(self.combiner_dyn_topic)

    def get_publish_rate(self):
        sinistra_result = self.sinistra_rate.get_hz(self.sinistra_topic)
        dextra_result = self.dextra_rate.get_hz(self.dextra_topic)
        sinistra_rate = 0.0 if sinistra_result is None else sinistra_result[0]
        dextra_rate = 0.0 if dextra_result is None else dextra_result[0]
        return sinistra_rate, dextra_rate

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
            sinistra_rate, dextra_rate = self.get_publish_rate()
            if not parameters_set and (sinistra_rate > 0.0 or dextra_rate > 0.0):
                self.set_parameters()
                parameters_set = True

            if not (self.sinistra_min_rate_threshold <= sinistra_rate <= self.sinistra_max_rate_threshold and
                    self.dextra_min_rate_threshold <= dextra_rate <= self.dextra_max_rate_threshold):
                rospy.logwarn_throttle(2.0, 
                    "LIDARs are not publishing within the threshold. " \
                    "sinistra (%0.1f..%0.1f): %0.2f. dextra (%0.1f..%0.1f): %0.2f" % (
                        self.sinistra_min_rate_threshold, self.sinistra_max_rate_threshold, sinistra_rate,
                        self.dextra_min_rate_threshold, self.dextra_max_rate_threshold, dextra_rate)
                )


if __name__ == "__main__":
    node = TJ2LidarWatcher()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
