import math
import numpy as np
import tf.transformations
from tj2_tools.robot_state import Pose2d
from matplotlib import pyplot as plt

fig = plt.figure(1)
ax = fig.add_subplot()

def plot_pose(pose, name, arrow_length=0.1):
    ax.plot([pose.x], [pose.y], 'x', label=name)
    angle = pose.theta
    dx = arrow_length * math.cos(angle)
    dy = arrow_length * math.sin(angle)
    ax.arrow(pose.x, pose.y, dx, dy, width=arrow_length * 0.5)


waypoint_pose2d = Pose2d(0.0, 0.0, 0.0)
# waypoint_pose2d = Pose2d(2.0, 2.0, 3.14)
tag_base_pose2d = Pose2d(x=0.1396, y=-0.8446, theta=-1.7059)
# tag_base_pose2d = Pose2d(1.0, 0.0, 0.0)

odom_pose2d = Pose2d(x=-0.2370, y=-0.0831, theta=0.2229)
# odom_pose2d = Pose2d(0.5, 0.0, 0.0)

map_to_waypoint_tf = waypoint_pose2d.to_transform_matrix()
base_to_tag_tf = tag_base_pose2d.to_transform_matrix()
tag_to_base_tf = tf.transformations.inverse_matrix(base_to_tag_tf)
map_to_base_tf = map_to_waypoint_tf @ tag_to_base_tf
robot_global_pose2d = Pose2d.from_transform_matrix(map_to_base_tf)

print(robot_global_pose2d)
print(Pose2d(x=-0.8181, y=-0.2521, theta=1.7059))

base_to_map_tf = tf.transformations.inverse_matrix(map_to_base_tf)
odom_to_base_tf = odom_pose2d.to_transform_matrix()
odom_to_map_tf = odom_to_base_tf @ base_to_map_tf
map_to_odom_tf = tf.transformations.inverse_matrix(odom_to_map_tf)
map_to_odom_pose2d = Pose2d.from_transform_matrix(map_to_odom_tf)

print(map_to_odom_pose2d)
print(Pose2d(x=-0.8799, y=-0.0104, theta=1.4851))

plot_pose(robot_global_pose2d, "robot")
plot_pose(waypoint_pose2d, "waypoint")
plot_pose(map_to_odom_pose2d, "odom")
plot_pose(Pose2d(), "map")

xmin, xmax = ax.get_xlim()
ymin, ymax = ax.get_ylim()

view_limit = np.max(np.abs(np.array([xmin, xmax, ymin, ymax])))

view_limit += 0.5

ax.set_xlim(-view_limit, view_limit)
ax.set_ylim(-view_limit, view_limit)

plt.legend()
plt.show()
