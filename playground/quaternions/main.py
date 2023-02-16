import math
import tf_conversions
from tf.transformations import quaternion_multiply

def euler_from_quaternion(x, y, z, w):
    angles = tf_conversions.transformations.euler_from_quaternion((x, y, z, w))
    print("%0.4f  %0.4f  %0.4f" % tuple(map(math.degrees, angles)))

def quaternion_from_euler(roll, pitch, yaw):
    print("roll x  pitch y   yaw z")
    print("%0.4f  %0.4f  %0.4f" % (roll, pitch, yaw))
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    quat = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
    print("     x       y       z       w")
    print("%0.4f  %0.4f  %0.4f  %0.4f" % tuple(quat))
    return quat

quaternion_from_euler(90.0, 90.0, 0.0)
# print(quaternion_multiply((0.5, 0.5, -0.5, 0.5), quaternion_from_euler(180.0, 0.0, 0.0)))
