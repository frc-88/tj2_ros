import math
import tf_conversions


def euler_from_quaternion(x, y, z, w):
    print(tf_conversions.transformations.euler_from_quaternion((x, y, z, w)))

def quaternion_from_euler(roll, pitch, yaw):
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    print(tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw))

quaternion_from_euler(70.0, 10.0, 90.0)
