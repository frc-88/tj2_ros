#!/usr/bin/python3
import rospy
import math

import tf2_ros
import tf_conversions

from std_msgs.msg import Float64
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from networktables import NetworkTables

from tj2_networktables.srv import OdomReset, OdomResetResponse

from robot_state import Pose2d


class TJ2NetworkTables(object):
    def __init__(self):
        self.node_name = "tj2_networktables"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.nt_host = rospy.get_param("~nt_host", "10.0.88.2")
        
        NetworkTables.initialize(server=self.nt_host)
        self.nt = NetworkTables.getTable("coprocessor")

        self.publish_odom_tf = rospy.get_param("~publish_odom_tf", False)
        self.base_frame_name = rospy.get_param("~base_frame", "base_link")
        self.odom_frame_name = rospy.get_param("~odom_frame", "odom")

        self.remote_units_conversion = rospy.get_param("~remote_units_conversion", 0.3048)  # WPILib uses a mix of meters, feet, inches...

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

        self.match_time_pub = rospy.Publisher("match_time", Float64, queue_size=5)
        self.is_autonomous_pub = rospy.Publisher("is_autonomous", Bool, queue_size=5)

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = self.odom_frame_name
        self.tf_msg.child_frame_id = self.base_frame_name

        self.driver_station_table_key = "host/DriverStation"
        self.odom_table_key = "host/odom"

        self.fms_flag_ignored = True

        self.remote_start_time = 0.0
        self.local_start_time = 0.0
        self.prev_timestamp = 0.0
        self.prev_odom_timestamp = 0.0

        self.start_pose = Pose2d()
        self.robot_pose = Pose2d()
        self.robot_vel = Pose2d()
        
        self.clock_rate = rospy.Rate(50.0)

        self.odom_reset_service_name = "odom_reset_service"
        rospy.loginfo("Setting up service %s" % self.odom_reset_service_name)
        self.odom_reset_service_name_srv = rospy.Service(self.odom_reset_service_name, OdomReset, self.odom_reset_callback)
        rospy.loginfo("%s service is ready" % self.odom_reset_service_name)

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        # self.wait_for_time()

        while not rospy.is_shutdown():
            if self.remote_start_time == 0.0:
                self.init_remote_time()
            self.clock_rate.sleep()
            self.publish_odom()
            self.publish_driver_station()
            self.update_heartbeat()
    
    def publish_driver_station(self):
        is_fms_attached = self.nt.getEntry(self.driver_station_table_key + "/isFMSAttached").getBoolean(False)
        is_autonomous = self.nt.getEntry(self.driver_station_table_key + "/isAutonomous").getBoolean(True)

        if self.fms_flag_ignored or is_fms_attached:
            match_time = self.nt.getEntry(self.driver_station_table_key + "/getMatchTime").getDouble(-1.0)
            self.match_time_pub.publish(match_time)
            self.is_autonomous_pub.publish(is_autonomous)
        else:
            self.match_time_pub.publish(-1.0)

    def publish_odom(self):
        timestamp = self.get_remote_time_as_local()
        if self.prev_odom_timestamp == timestamp:
            return
        self.prev_odom_timestamp = timestamp
        
        x = self.nt.getEntry(self.odom_table_key + "/xPosition").getDouble(0.0)
        y = self.nt.getEntry(self.odom_table_key + "/yPosition").getDouble(0.0)
        theta = self.nt.getEntry(self.odom_table_key + "/theta").getDouble(0.0)
        vx = self.nt.getEntry(self.odom_table_key + "/xVelocity").getDouble(0.0)
        vy = self.nt.getEntry(self.odom_table_key + "/yVelocity").getDouble(0.0)
        vt = self.nt.getEntry(self.odom_table_key + "/thetaVelocity").getDouble(0.0)

        # x *= self.remote_units_conversion
        # y *= self.remote_units_conversion
        theta = math.radians(theta)
        # vx *= self.remote_units_conversion
        # vy *= self.remote_units_conversion
        vt = math.radians(vt)

        self.robot_pose.x = x
        self.robot_pose.y = y
        self.robot_pose.theta = theta

        adj_robot_pose = self.robot_pose - self.start_pose

        quaternion = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, adj_robot_pose.theta)

        # ros_time = rospy.Time(timestamp)
        ros_time = rospy.Time.now()

        self.odom_msg.header.stamp = ros_time
        self.odom_msg.pose.pose.position.x = adj_robot_pose.x
        self.odom_msg.pose.pose.position.y = adj_robot_pose.y
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
            self.tf_msg.transform.translation.x = adj_robot_pose.x
            self.tf_msg.transform.translation.y = adj_robot_pose.y
            self.tf_msg.transform.rotation.x = quaternion[0]
            self.tf_msg.transform.rotation.y = quaternion[1]
            self.tf_msg.transform.rotation.z = quaternion[2]
            self.tf_msg.transform.rotation.w = quaternion[3]

            self.br.sendTransform(self.tf_msg)
    
    def update_heartbeat(self):
        self.nt.getEntry("client/timestamp").setNumber(self.get_local_time_as_remote())

    def get_remote_time(self):
        """
        Gets RoboRIO's timestamp based on the networktables entry in seconds
        """
        return self.nt.getEntry("host/timestamp").getDouble(0.0) * 1E-6
    
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
            self.init_remote_time()
            self.clock_rate.sleep()
            if rospy.is_shutdown():
                return
        rospy.loginfo("Remote time found")
    
    def init_remote_time(self):
        self.remote_start_time = self.get_remote_time()
        self.local_start_time = self.get_local_time()
    
    def get_remote_time_as_local(self):
        """
        Gets the RoboRIO's time relative to ROS time epoch
        """
        remote_timestamp = self.get_remote_time()
        self.check_time_offsets(remote_timestamp)
        return remote_timestamp - self.remote_start_time + self.local_start_time

    def odom_reset_callback(self, req):
        self.start_pose = Pose2d.from_state(self.robot_pose)
        self.start_pose.x += req.x
        self.start_pose.y += req.y
        self.start_pose.theta += req.t
        rospy.loginfo("Resetting odometry to x: %0.3f, y: %0.3f, theta: %0.3f" % (req.x, req.y, req.t))

        return OdomResetResponse(True)

if __name__ == "__main__":
    node = TJ2NetworkTables()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
