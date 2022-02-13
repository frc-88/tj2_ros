import math
import tf_conversions


def euler_from_quaternion(x, y, z, w):
    print(tf_conversions.transformations.euler_from_quaternion((x, y, z, w)))

def quaternion_from_euler(roll, pitch, yaw):
    print(roll, pitch, yaw)
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    quat = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
    print("%0.4f  %0.4f  %0.4f  %0.4f" % tuple(quat))

# euler_from_quaternion(0, 0, 1, 0)
# quaternion_from_euler(-90.0 + 3.0, 8.0, 90.0)
quaternion_from_euler(0.0, 42.5124239 - 90, 180.0)
