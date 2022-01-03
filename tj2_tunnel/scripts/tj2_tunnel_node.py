#!/usr/bin/python3
import rospy
import math
import time

import tf2_ros
import tf_conversions

from std_msgs.msg import Float64
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

from tj2_tools.robot_state import Pose2d
from tj2_tools.tunnel.client import TunnelClient


class TJ2Tunnel(TunnelClient):
    def __init__(self):
        self.node_name = "tj2_tunnel"
        rospy.init_node(
            self.node_name,
            # disable_signals=True,
            # log_level=rospy.DEBUG
        )

        self.host = rospy.get_param("~host", "10.0.88.2")
        self.port = rospy.get_param("~port", 5800)
        
        self.publish_odom_tf = rospy.get_param("~publish_odom_tf", False)
        self.base_frame_name = rospy.get_param("~base_frame", "base_link")
        self.odom_frame_name = rospy.get_param("~odom_frame", "odom")

        self.remote_linear_units_conversion = rospy.get_param("~remote_linear_units_conversion", 0.3048)  # WPILib uses a mix of meters, feet, inches...
        self.remote_angular_units_conversion = rospy.get_param("~remote_angular_units_conversion", math.pi / 180.0)  # WPILib uses a mix of radians and degrees...
        self.cmd_vel_timeout = rospy.get_param("~cmd_vel_timeout", 0.5)
        self.min_linear_x_cmd = rospy.get_param("~min_linear_x_cmd", 0.05)
        self.min_linear_y_cmd = rospy.get_param("~min_linear_y_cmd", 0.05)
        self.min_angular_z_cmd = rospy.get_param("~min_angular_z_cmd", 0.1)
        self.zero_epsilon = rospy.get_param("~zero_epsilon", 0.001)

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame_name
        self.odom_msg.child_frame_id = self.base_frame_name

        self.odom_msg.pose.covariance = [
            5e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 5e-2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 5e-2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 5e-2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 5e-2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 5e-2
        ]
        self.odom_msg.twist.covariance = [
            10e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 10e-2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 10e-2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 10e-2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 10e-2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 10e-2
        ]

        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=50)

        self.ping_pub = rospy.Publisher("ping", Float64, queue_size=50)

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = self.odom_frame_name
        self.tf_msg.child_frame_id = self.base_frame_name

        self.prev_twist_timestamp = rospy.Time.now()
        self.prev_odom_timestamp = rospy.Time.now()

        self.robot_vel = Pose2d()
        
        self.clock_rate = rospy.Rate(100.0)
        self.ping_timer = rospy.Timer(rospy.Duration(1.0), self.ping_callback)

        super(TJ2Tunnel, self).__init__(self.host, self.port)
        rospy.loginfo("%s init complete" % self.node_name)

    def packet_callback(self, result):
        category = result.category
        if category == "ping":
            msg = Float64()
            ping_time = result.get_double()
            dt = self.get_local_time() - ping_time
            msg.data = dt
            rospy.logdebug("Publishing ping time: %s. (Return time: %s)", dt, ping_time);
            self.ping_pub.publish(msg)

        elif category == "odom":
            # rospy.loginfo("Odometry. x=%0.4f, y=%0.4f, t=%0.4f, vx=%0.4f, vy=%0.4f, vt=%0.4f" % tuple(data))
            data = tuple([result.get_double() for _ in range(6)])
            self.publish_odom(result.recv_time, *data)

    def run(self):
        try:
            self.start()
            while not rospy.is_shutdown():
                self.clock_rate.sleep()
                self.update()
                # self.publish_cmd_vel()
        finally:
            self.stop()

    def twist_callback(self, msg):
        self.robot_vel.x = -msg.linear.x
        self.robot_vel.y = -msg.linear.y
        self.robot_vel.theta = -msg.angular.z

        if self.zero_epsilon < abs(self.robot_vel.x) < self.min_linear_x_cmd:
            self.robot_vel.x = self.min_linear_x_cmd
        if self.zero_epsilon < abs(self.robot_vel.y) < self.min_linear_y_cmd:
            self.robot_vel.y = self.min_linear_y_cmd
        if self.zero_epsilon < abs(self.robot_vel.theta) < self.min_angular_z_cmd:
            self.robot_vel.theta = self.min_angular_z_cmd
        
        if abs(self.robot_vel.x) < self.zero_epsilon:
            self.robot_vel.x = 0.0
        if abs(self.robot_vel.y) < self.zero_epsilon:
            self.robot_vel.y = 0.0
        if abs(self.robot_vel.theta) < self.zero_epsilon:
            self.robot_vel.theta = 0.0
        
        self.prev_twist_timestamp = rospy.Time.now()
    
    def publish_cmd_vel(self, ignore_timeout=False):
        dt = (rospy.Time.now() - self.prev_twist_timestamp).to_sec()
        if not ignore_timeout and dt > self.cmd_vel_timeout:
            return
        self.write("cmd",
            math.degrees(self.robot_vel.heading() % (2 * math.pi)),
            self.robot_vel.magnitude() / self.remote_linear_units_conversion,
            math.degrees(self.robot_vel.theta)
        )

    def publish_odom(self, timestamp, x, y, theta, vx, vy, vt):
        if self.prev_odom_timestamp == timestamp:
            return
        self.prev_odom_timestamp = timestamp

        x *= self.remote_linear_units_conversion
        y *= self.remote_linear_units_conversion
        theta *= self.remote_angular_units_conversion
        vx *= self.remote_linear_units_conversion
        vy *= self.remote_linear_units_conversion
        vt *= self.remote_angular_units_conversion

        quaternion = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, theta)

        # ros_time = rospy.Time(timestamp)
        ros_time = rospy.Time.now()

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
    
    def ping_callback(self, timer):
        rospy.logdebug("Writing ping")
        self.write("ping", self.get_local_time())

    def get_local_time(self):
        return rospy.Time.now().to_sec()


if __name__ == "__main__":
    node = TJ2Tunnel()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
