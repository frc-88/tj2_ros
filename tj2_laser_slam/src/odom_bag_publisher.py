#!/usr/bin/python3
import rospy

import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class OdomBagPublisher(object):
    def __init__(self):
        self.node_name = "odom_bag_publisher"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=50)

        self.x_scale = rospy.get_param("~x_scale", 1.0)
        self.y_scale = rospy.get_param("~y_scale", 1.0)
        self.z_scale = rospy.get_param("~z_scale", 1.0)
        self.use_bag_timestamp = rospy.get_param("~use_bag_timestamp", False)

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()

        self.prev_msg = None
        self.num_skipped = 0
        self.num_published = 0

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        rospy.spin()

    def is_msg_equal(self, other):
        if self.prev_msg is None:
            return False
        return self.prev_msg.pose.pose == other.pose.pose
    
    def odom_callback(self, msg):
        if self.is_msg_equal(msg):
            self.num_skipped += 1
            return
        self.prev_msg = msg
        self.num_published += 1


        self.tf_msg.header.frame_id = msg.header.frame_id
        self.tf_msg.child_frame_id = msg.child_frame_id
        if self.use_bag_timestamp:
            self.tf_msg.header.stamp = msg.header.stamp
        else:
            self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_msg.transform.translation.x = msg.pose.pose.position.x * self.x_scale
        self.tf_msg.transform.translation.y = msg.pose.pose.position.y * self.y_scale
        self.tf_msg.transform.translation.z = msg.pose.pose.position.z * self.z_scale
        self.tf_msg.transform.rotation.x = msg.pose.pose.orientation.x
        self.tf_msg.transform.rotation.y = msg.pose.pose.orientation.y
        self.tf_msg.transform.rotation.z = msg.pose.pose.orientation.z
        self.tf_msg.transform.rotation.w = msg.pose.pose.orientation.w

        self.br.sendTransform(self.tf_msg)
    
    def close(self):
        rospy.loginfo("%s of %s messages skipped" % (self.num_skipped, self.num_published))


if __name__ == "__main__":
    node = OdomBagPublisher()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.close()
        rospy.loginfo("Exiting %s node" % node.node_name)
