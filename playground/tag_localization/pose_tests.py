import math
from tj2_tools.robot_state import Pose2d
from matplotlib import pyplot as plt

def plot_pose(pose, arrow_length=0.25):
    plt.plot([pose.x], [pose.y], 'x')
    angle = pose.heading() + pose.theta
    dx = arrow_length * math.cos(angle)
    dy = arrow_length * math.sin(angle)
    plt.arrow(pose.x, pose.y, dx, dy)

base_to_global2d = Pose2d(0.5, 0.5, math.radians(45.0))
base_to_odom2d = Pose2d(0.4, 0.5, math.radians(0.0))

odom_to_global2d = base_to_global2d.relative_to(base_to_odom2d)
print("base to odom  ", base_to_odom2d, math.degrees(base_to_odom2d.theta))
print("odom to global", odom_to_global2d, math.degrees(odom_to_global2d.theta))
print("base to global", base_to_global2d, math.degrees(base_to_global2d.theta))
print("calc          ", odom_to_global2d.transform_by(base_to_odom2d))

# p0 = Pose2d(0.0, 0.0, 0.0)
# p1 = p0.transform_by(base_to_odom2d)

plot_pose(base_to_global2d)
plot_pose(base_to_odom2d)

plt.xlim(-2.0, 2.0)
plt.ylim(-2.0, 2.0)
plt.show()