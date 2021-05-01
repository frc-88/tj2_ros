#!/usr/bin/python3
import rospy
import math

import tf2_ros
import tf_conversions

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

from networktables import NetworkTables

from tj2_networktables.msg import SwerveModule



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

        self.remote_units_conversion = rospy.get_param("~remote_units_conversion", 0.3048)  # WPILib uses a mix of meters, feet, inches...

        self.num_modules = rospy.get_param("~num_modules", 4)

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame_name
        self.odom_msg.child_frame_id = self.base_frame_name

        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=50)

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        self.module_states = []
        self.module_publishers = []
        for module_num in range(self.num_modules):
            self.module_states.append(SwerveModule())
            self.module_publishers.append(rospy.Publisher("swerve_modules/%s" % (module_num + 1), SwerveModule, queue_size=10))

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = self.odom_frame_name
        self.tf_msg.child_frame_id = self.base_frame_name

        self.remote_start_time = 0.0
        self.local_start_time = 0.0
        self.prev_timestamp = 0.0
        self.prev_odom_timestamp = 0.0
        
        self.clock_rate = rospy.Rate(50.0)

        rospy.loginfo("Network tables init complete")

    def run(self):
        self.wait_for_time()

        while not rospy.is_shutdown():
            self.clock_rate.sleep()            
            self.publish_odom()
    
    def twist_callback(self, msg):
        remote_time = self.get_local_time_as_remote()
        self.nt.putNumber("command/time", remote_time)
        self.nt.putNumber("command/linear_x", msg.linear.x / self.remote_units_conversion)
        self.nt.putNumber("command/linear_y", msg.linear.y / self.remote_units_conversion)
        self.nt.putNumber("command/angular_z", math.degrees(msg.angular.z))
    
    def publish_modules(self):
        for module_num in range(self.num_modules):
            wheel_velocity = self.nt.getEntry("modules/%s/wheel" % (module_num + 1)).getDouble(0.0)
            wheel_velocity *= self.remote_units_conversion
            azimuth = self.nt.getEntry("modules/%s/azimuth" % (module_num + 1)).getDouble(0.0)
            azimuth = math.radians(azimuth)
            
            module_state = self.module_states[module_num]
            module_state.velocity = wheel_velocity
            module_state.azimuth = azimuth

            publisher = self.module_publishers[module_num]
            publisher.publish(module_state)

    def publish_odom(self):
        timestamp = self.get_remote_time_as_local()
        if self.prev_odom_timestamp == timestamp:
            return
        self.prev_odom_timestamp = timestamp
        
        x = self.nt.getEntry("odom/x").getDouble(0.0)
        y = self.nt.getEntry("odom/y").getDouble(0.0)
        theta = self.nt.getEntry("odom/t").getDouble(0.0)
        vx = self.nt.getEntry("odom/vx").getDouble(0.0)
        vy = self.nt.getEntry("odom/vy").getDouble(0.0)
        vt = self.nt.getEntry("odom/vt").getDouble(0.0)

        x *= self.remote_units_conversion
        y *= self.remote_units_conversion
        # theta = math.radians(theta)
        vx *= self.remote_units_conversion
        vy *= self.remote_units_conversion
        # vt = math.radians(vt)

        quaternion = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, theta)

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
        
        self.publish_modules()
        
    def get_remote_time(self):
        """
        Gets RoboRIO's timestamp based on the Swerve/odom/time entry in seconds
        """
        return self.nt.getEntry("odom/time").getDouble(0.0) * 1E-6
    
    def get_local_time(self):
        return rospy.Time.now().to_sec()

    def check_time_offsets(self, remote_timestamp):
        if remote_timestamp < self.prev_timestamp:  # remote has restarted
            self.remote_start_time = remote_timestamp
            self.local_start_time = self.get_local_time()
            rospy.loginfo("Remote clock jumped back. Resetting offset")
        self.prev_timestamp = remote_timestamp

    def get_local_time_as_remote(self):
        local_timestamp = self.get_local_time()
        remote_timestamp = self.get_remote_time()
        self.check_time_offsets(remote_timestamp)
        remote_time = local_timestamp - self.local_start_time + self.remote_start_time
        remote_time *= 1E6
        return remote_time

    def wait_for_time(self):
        rospy.loginfo("Waiting for remote time")
        while self.remote_start_time == 0.0:
            self.remote_start_time = self.get_remote_time()
            self.clock_rate.sleep()
            if rospy.is_shutdown():
                return
        rospy.loginfo("Remote time found")
        self.local_start_time = self.get_local_time()
    
    def get_remote_time_as_local(self):
        """
        Gets the RoboRIO's time relative to ROS time epoch
        """
        remote_timestamp = self.get_remote_time()
        self.check_time_offsets(remote_timestamp)
        return remote_timestamp - self.remote_start_time + self.local_start_time



if __name__ == "__main__":
    node = TJ2NetworkTables()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
