import math
import tf_conversions
from scipy.spatial.transform import Rotation


def euler_from_quaternion(x, y, z, w):
    angles = tf_conversions.transformations.euler_from_quaternion((x, y, z, w))
    print_euler(tuple(map(math.degrees, angles)))


def quaternion_from_euler(roll, pitch, yaw):
    # print_euler((roll, pitch, yaw))
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    quat = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
    # print_quaternion(quat)
    return quat


def print_euler(euler):
    roll, pitch, yaw = euler
    # print("roll x  pitch y   yaw z")
    print("%0.4f  %0.4f  %0.4f" % (roll, pitch, yaw))


def print_quaternion(quat):
    # print("     x       y       z       w")
    # print("qx: %0.4f, qy: %0.4f, qz: %0.4f, qw: %0.4f" % tuple(quat))
    print("%0.4f  %0.4f  %0.4f  %0.4f" % tuple(quat))


def quaternion_multiply(rotate_quat, quat):
    rotate_mat = Rotation.from_quat(rotate_quat)  # type: ignore
    source_mat = Rotation.from_quat(quat)  # type: ignore
    rotated = source_mat * rotate_mat
    return rotated.as_quat()


# euler_from_quaternion(0.0000, 0.0000, -0.7071, 0.7071)
# euler_from_quaternion(-0.0145, -0.0063, 0.2245, 0.9743)
# euler_from_quaternion(-0.0147, 0.0058, -0.5302, 0.8477)
# euler_from_quaternion(0.0872,  -0.0000,  0.9962,  0.0000)
# quaternion_from_euler(0.0000,  -13.5,  0.0000)
# quaternion_from_euler(0.0, -42.5124239, 180.0)
# quaternion_from_euler(90.0, 0.0, 90.0)

camera_optical_rotation = (0.5, -0.5, -0.5, -0.5)


# print(quaternion_multiply((0.5, 0.5, 0.5, 0.5), quaternion_from_euler(0.0, 180.0, 0.0)))
print_quaternion(quaternion_from_euler(180.0000, -10.0, 0.0000))
print_quaternion(quaternion_from_euler(180.0000, -10.0, 90.0000))
print_quaternion(quaternion_from_euler(180.0000, -10.0, 180.0000))
print_quaternion(quaternion_from_euler(180.0000, -10.0, 270.0000))

# print_quaternion(quaternion_from_euler(-90.0000, 0.0000, -90.0000))
