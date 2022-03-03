#!/usr/bin/python3
import rospy
import math
import tf_conversions


from geometry_msgs.msg import PoseWithCovarianceStamped


def euler_from_quaternion(x, y, z, w):
    return tf_conversions.transformations.euler_from_quaternion((x, y, z, w))


def quaternion_from_euler(roll, pitch, yaw):
    print(roll, pitch, yaw)
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    return tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
    print("%0.4f  %0.4f  %0.4f  %0.4f" % tuple(quat))


class QuaternionListener:
    def __init__(self):
        self.node_name = "quaternion_listener"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.subscriber = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.callback, queue_size=50)

        rospy.loginfo("%s init complete" % self.node_name)

    def run(self):
        rospy.spin()

    def callback(self, msg):
        angles = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # data = [
        #     msg.pose.pose.position.x,
        #     msg.pose.pose.position.y,
        #     msg.pose.pose.position.z,
        # ]
        # data += list(map(math.degrees, angles))
        # print("Position: %0.4f, %0.4f, %0.4f\tAngles: %0.4f  %0.4f  %0.4f" % tuple(data))
        print("(x, y, theta) [%0.4f, %0.4f, %0.4f]" % (msg.pose.pose.position.x, msg.pose.pose.position.y, angles[2]))


if __name__ == "__main__":
    node = QuaternionListener()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
