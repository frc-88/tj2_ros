#!/usr/bin/env python3
import math
import rospy
from message_filters import Subscriber
from message_filters import ApproximateTimeSynchronizer

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from tf.transformations import euler_from_quaternion


class ImuMerger:
    def __init__(self):
        self.name = "imu_merger"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)
        self.accel_sub = Subscriber("accel/sample", Imu)
        self.gyro_sub = Subscriber("gyro/sample", Imu)
        self.time_sync = ApproximateTimeSynchronizer([self.accel_sub, self.gyro_sub], queue_size=10, slop=0.1)
        self.time_sync.registerCallback(self.imu_sync_callback)

        self.sync_imu_pub = rospy.Publisher("imu", Imu, queue_size=10)
        self.camera_joint_pub = rospy.Publisher("/tj2/camera_joint", Float64, queue_size=10)

        self.filter_imu_sub = rospy.Subscriber("imu/filtered", Imu, self.imu_filtered_callback, queue_size=10)

        rospy.loginfo("%s init complete" % self.name)
    
    def imu_sync_callback(self, accel_msg, gyro_msg):
        msg = accel_msg
        msg.header.frame_id = "camera_tilt_link"
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.angular_velocity = gyro_msg.angular_velocity
        msg.angular_velocity_covariance = gyro_msg.angular_velocity_covariance
        self.sync_imu_pub.publish(msg)
    
    def imu_filtered_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ])
        roll += math.pi / 2.0
        roll *= -1.0
        self.camera_joint_pub.publish(roll)

    def run(self):
        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":
    node = ImuMerger()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
