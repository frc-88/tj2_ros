#!/usr/bin/python3
import rospy
import math

import tf2_ros
import tf_conversions

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

from networktables import NetworkTables

from tj2_networktables.msg import SwerveModule
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

        NetworkTables.initialize(server="10.0.88.2")
        self.nt = NetworkTables.getTable("swerveLibrary")

        self.publish_odom_tf = rospy.get_param("~publish_odom_tf", False)
        self.base_frame_name = rospy.get_param("~base_frame", "base_link")
        self.odom_frame_name = rospy.get_param("~odom_frame", "odom")
        self.imu_frame_name = rospy.get_param("~imu_frame", "imu")

        self.remote_units_conversion = rospy.get_param("~remote_units_conversion", 0.3048)  # WPILib uses a mix of meters, feet, inches...
        self.cmd_vel_timeout = rospy.get_param("~cmd_vel_timeout", 0.5)
        self.min_linear_x_cmd = rospy.get_param("~min_linear_x_cmd", 0.05)
        self.min_linear_y_cmd = rospy.get_param("~min_linear_y_cmd", 0.05)
        self.min_angular_z_cmd = rospy.get_param("~min_angular_z_cmd", 0.1)
        self.zero_epsilon = rospy.get_param("~zero_epsilon", 0.001)

        self.num_modules = rospy.get_param("~num_modules", 4)

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame_name
        self.odom_msg.child_frame_id = self.base_frame_name

        self.odom_msg.pose.covariance = [
            5e+1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 5e+1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 5e+1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 5e+1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 5e+1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 5e+1
        ]
        self.odom_msg.twist.covariance = [
            10e+1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 10e+1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 10e+1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 10e+1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 10e+1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 10e+1
        ]

        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=50)

        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=50)
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.imu_frame_name
        self.imu_msg.orientation_covariance = [
            1e-5, 0.0, 0.0,
            0.0, 1e-5, 0.0,
            0.0, 0.0, 1e-5
        ]
        self.imu_msg.angular_velocity_covariance = [
            1e-5, 0.0, 0.0,
            0.0, 1e-5, 0.0,
            0.0, 0.0, 1e-5
        ]
        self.imu_msg.linear_acceleration_covariance = [
            5e-5, 0.0, 0.0,
            0.0, 5e-5, 0.0,
            0.0, 0.0, 5e-5
        ]

        self.module_states = []
        self.module_publishers = []
        for module_num in range(self.num_modules):
            self.module_states.append(SwerveModule())
            self.module_publishers.append(rospy.Publisher("swerve_modules/%s" % module_num, SwerveModule, queue_size=10))

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = self.odom_frame_name
        self.tf_msg.child_frame_id = self.base_frame_name

        self.remote_start_time = 0.0
        self.local_start_time = 0.0
        self.prev_timestamp = 0.0
        self.prev_odom_timestamp = 0.0
        self.prev_imu_timestamp = 0.0
        self.prev_twist_timestamp = rospy.Time.now()

        self.odom_table_key = "odometryState"
        self.command_table_key = "commands"
        self.module_table_key = "modules"
        self.imu_table_key = "gyro"

        self.start_pose = Pose2d()
        self.robot_pose = Pose2d()
        self.robot_vel = Pose2d()
        
        self.clock_rate = rospy.Rate(50.0)

        self.odom_reset_service_name = "odom_reset_service"
        rospy.loginfo("Setting up service %s" % self.odom_reset_service_name)
        self.odom_reset_service_name_srv = rospy.Service(self.odom_reset_service_name, OdomReset, self.odom_reset_callback)
        rospy.loginfo("%s service is ready" % self.odom_reset_service_name)

        rospy.loginfo("Network tables init complete")

    def run(self):
        self.wait_for_time()

        self.publish_cmd_vel(ignore_timeout=True)  # clear networktables cmd_vel once
        while not rospy.is_shutdown():
            self.clock_rate.sleep()            
            self.publish_odom()
            self.publish_cmd_vel()
            self.publish_imu()
    
    def twist_callback(self, msg):
        self.robot_vel.x = msg.linear.x
        self.robot_vel.y = msg.linear.y
        self.robot_vel.theta = msg.angular.z

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
        remote_time = self.get_local_time_as_remote()
        self.nt.putNumber(self.command_table_key + "/timestamp", remote_time)
        self.nt.putNumber(self.command_table_key + "/translationSpeed", self.robot_vel.x / self.remote_units_conversion)
        self.nt.putNumber(self.command_table_key + "/translationDirection", self.robot_vel.y / self.remote_units_conversion)
        self.nt.putNumber(self.command_table_key + "/rotationVelocity", math.degrees(self.robot_vel.theta))
        self.nt.putBoolean(self.command_table_key + "/isFieldCentric", False)

    def publish_modules(self):
        for module_num in range(self.num_modules):
            wheel_velocity = self.nt.getEntry(self.module_table_key + "/%s/wheelVelocity" % module_num).getDouble(0.0)
            wheel_velocity *= self.remote_units_conversion
            azimuth = self.nt.getEntry(self.module_table_key + "/%s/azimuthPosition" % module_num).getDouble(0.0)
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
        
        x = self.nt.getEntry(self.odom_table_key + "/xPosition").getDouble(0.0)
        y = self.nt.getEntry(self.odom_table_key + "/yPosition").getDouble(0.0)
        theta = self.nt.getEntry(self.odom_table_key + "/theta").getDouble(0.0)
        vx = self.nt.getEntry(self.odom_table_key + "/xVelocity").getDouble(0.0)
        vy = self.nt.getEntry(self.odom_table_key + "/yVelocity").getDouble(0.0)
        vt = self.nt.getEntry(self.odom_table_key + "/thetaVelocity").getDouble(0.0)

        x *= self.remote_units_conversion
        y *= self.remote_units_conversion
        theta = math.radians(theta)
        vx *= self.remote_units_conversion
        vy *= self.remote_units_conversion
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
        
        self.publish_modules()
    
    def publish_imu(self):
        timestamp = self.get_remote_time_as_local()
        if self.prev_imu_timestamp == timestamp:
            return
        self.prev_imu_timestamp = timestamp

        # ros_time = rospy.Time(timestamp)
        ros_time = rospy.Time.now()

        yaw = self.nt.getEntry(self.imu_table_key + "/yaw").getDouble(0.0)
        ang_vz = self.nt.getEntry(self.imu_table_key + "/yawRate").getDouble(0.0)
        ax = self.nt.getEntry(self.imu_table_key + "/accelX").getDouble(0.0)
        ay = self.nt.getEntry(self.imu_table_key + "/accelY").getDouble(0.0)

        quaternion = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, yaw)

        self.imu_msg.header.stamp = ros_time
        self.imu_msg.orientation.x = quaternion[0]
        self.imu_msg.orientation.y = quaternion[1]
        self.imu_msg.orientation.z = quaternion[2]
        self.imu_msg.orientation.w = quaternion[3]
        self.imu_msg.angular_velocity.z = ang_vz
        self.imu_msg.linear_acceleration.x = ax
        self.imu_msg.linear_acceleration.y = ay

        self.imu_pub.publish(self.imu_msg)
    
    def get_remote_time(self):
        """
        Gets RoboRIO's timestamp based on the Swerve/odom/time entry in seconds
        """
        return self.nt.getEntry("timestamp").getDouble(0.0) * 1E-6
    
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
