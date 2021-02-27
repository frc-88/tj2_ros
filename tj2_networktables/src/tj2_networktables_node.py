#!/usr/bin/python
import rospy

import tf2_ros
import tf_conversions

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import geometry_msgs.msg

from networktables import NetworkTables


class TJ2NetworkTables(object):
    def __init__(self):
        self.node_name = "tj2_networktables"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        NetworkTables.initialize(server="10.0.88.2")
        self.nt = NetworkTables.getTable("Swerve")

        self.publish_odom_tf = rospy.get_param("~publish_odom_tf", False)
        self.base_frame_name = rospy.get_param("~base_frame", "base_link")
        self.odom_frame_name = rospy.get_param("~odom_frame", "odom")

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame_name
        self.odom_msg.child_frame_id = self.base_frame_name

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = geometry_msgs.msg.TransformStamped()
        self.tf_msg.header.frame_id = self.odom_frame_name
        self.tf_msg.child_frame_id = self.base_frame_name

        self.remote_start_time = 0.0
        self.local_start_time = 0.0
        self.prev_timestamp = 0.0
        
        self.clock_rate = rospy.Rate(50.0)

        rospy.loginfo("Network tables init complete")

    def run(self):
        self.wait_for_time()

        while not rospy.is_shutdown():
            self.clock_rate.sleep()
            timestamp = self.get_time()
            self.nt.putNumber("command/time", rospy.Time.now().to_sec())
            
            self.publish_odom(timestamp)
    
    def publish_odom(self, timestamp):
        x = self.nt.getEntry("odom/x").getDouble(0.0)
        y = self.nt.getEntry("odom/y").getDouble(0.0)
        t = self.nt.getEntry("odom/t").getDouble(0.0)
        vx = self.nt.getEntry("odom/vx").getDouble(0.0)
        vy = self.nt.getEntry("odom/vy").getDouble(0.0)
        vt = self.nt.getEntry("odom/vt").getDouble(0.0)

        quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, t)

        ros_time = rospy.Time(timestamp)

        self.odom_msg.header.stamp = ros_time
        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y
        self.odom_msg.pose.pose.orientation.x = quaternion[0]
        self.odom_msg.pose.pose.orientation.y = quaternion[1]
        self.odom_msg.pose.pose.orientation.z = quaternion[2]
        self.odom_msg.pose.pose.orientation.w = quaternion[3]
        
        self.odom_msg.twist.twist.linear.x = vx
        self.odom_msg.twist.twist.linear.y = vy
        self.odom_msg.twist.twist.angular.z = vt

        self.odom_pub.publish(self.odom_msg)

        if self.publish_odom_tf:
            self.tf_msg.header.stamp = ros_time
            self.tf_msg.transform.translation.x = x
            self.tf_msg.transform.translation.y = y
            self.tf_msg.transform.rotation.x = quaternion[0]
            self.tf_msg.transform.rotation.y = quaternion[1]
            self.tf_msg.transform.rotation.z = quaternion[2]
            self.tf_msg.transform.rotation.w = quaternion[3]

            self.br.sendTransform(self.tf_msg)
        
    def get_remote_time(self):
        return self.nt.getEntry("odom/time").getDouble(0.0)
    
    def wait_for_time(self):
        rospy.loginfo("Waiting for remote time")
        while self.remote_start_time == 0.0:
            self.remote_start_time = self.get_remote_time()
            self.clock_rate.sleep()
            if rospy.is_shutdown():
                return
        rospy.loginfo("Remote time found")
        self.local_start_time = rospy.Time.now().to_sec()
    
    def get_time(self):
        timestamp = self.get_remote_time()
        if timestamp < self.prev_timestamp:  # remote has restarted
            self.remote_start_time = timestamp
            self.local_start_time = rospy.Time.now().to_sec()
            rospy.loginfo("Remote clock jumped back. Resetting offset")
        self.prev_timestamp = timestamp
        return timestamp - self.remote_start_time + self.local_start_time



if __name__ == "__main__":
    node = TJ2NetworkTables()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
