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

        self.expected_front_rate = rospy.get_param("~expected_front_rate", 10.0)
        self.expected_rear_rate = rospy.get_param("~expected_rear_rate", 10.0)
        self.min_rate_offset = rospy.get_param("~rate_band", 5.0)
        self.front_min_rate_threshold = max(0.0, self.expected_front_rate - self.min_rate_offset)
        self.front_max_rate_threshold = max(0.0, self.expected_front_rate + self.min_rate_offset)
        self.rear_min_rate_threshold = max(0.0, self.expected_rear_rate - self.min_rate_offset)
        self.rear_max_rate_threshold = max(0.0, self.expected_rear_rate + self.min_rate_offset)
        self.combiner_config = rospy.get_param("~combiner_config", None)

        self.front_rate = rostopic.ROSTopicHz(15)
        self.front_topic = "/front_laser/scan"
        rospy.Subscriber(self.front_topic, rospy.AnyMsg, self.front_rate.callback_hz, callback_args=self.front_topic, queue_size=10)

        self.rear_rate = rostopic.ROSTopicHz(15)
        self.rear_topic = "/rear_laser/scan"
        rospy.Subscriber(self.rear_topic, rospy.AnyMsg, self.rear_rate.callback_hz, callback_args=self.rear_topic, queue_size=10)

        self.combiner_client = None
        self.combiner_dyn_topic = "/laserscan_multi_merger"

        rospy.loginfo("%s init complete" % self.node_name)

    def init_dynamic_clients(self):
        if self.combiner_client is None:
            self.combiner_client = DynamicClient(self.combiner_dyn_topic)

    def get_publish_rate(self):
        front_result = self.front_rate.get_hz(self.front_topic)
        rear_result = self.rear_rate.get_hz(self.rear_topic)
        front_rate = 0.0 if front_result is None else front_result[0]
        rear_rate = 0.0 if rear_result is None else rear_result[0]
        return front_rate, rear_rate

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
            front_rate, rear_rate = self.get_publish_rate()
            if not parameters_set and (front_rate > 0.0 or rear_rate > 0.0):
                self.set_parameters()
                parameters_set = True

            if not (self.front_min_rate_threshold <= front_rate <= self.front_max_rate_threshold and
                    self.rear_min_rate_threshold <= rear_rate <= self.rear_max_rate_threshold):
                rospy.logwarn_throttle(2.0, 
                    "LIDARs are not publishing within the threshold. " \
                    "Front (%0.1f..%0.1f): %0.2f. Rear (%0.1f..%0.1f): %0.2f" % (
                        self.front_min_rate_threshold, self.front_max_rate_threshold, front_rate,
                        self.rear_min_rate_threshold, self.rear_max_rate_threshold, rear_rate)
                )


if __name__ == "__main__":
    node = TJ2LidarWatcher()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
