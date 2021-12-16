#!/usr/bin/python3
import rospy
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


class TopicTimer:
    def __init__(self):
        self.node_name = "topic_timer"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.topics = {
            "/limelight/color/image_raw": Image,
            "/limelight/aligned_depth_to_color/image_raw": Image,
            "/limelight/aligned_depth_to_color/camera_info": CameraInfo,
        }

        self.subscribers = {}
        self.timers = {}
        for topic, msg_type in self.topics.items():
            self.subscribers[topic] = rospy.Subscriber(topic, msg_type, lambda msg, topic=topic: self.callback(topic, msg), queue_size=50)
            self.timers[topic] = 0

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        clock_rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            report = ""
            for topic, timer in self.timers.items():
                if timer == 0:
                    continue
                offset = rospy.Time.now() - timer
                report += "%s is %0.4fs behind\n" % (topic, offset.to_sec())
            rospy.loginfo_throttle(0.25, report)
            clock_rate.sleep()

    def callback(self, topic, msg):
        self.timers[topic] = msg.header.stamp

if __name__ == "__main__":
    node = TopicTimer()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
