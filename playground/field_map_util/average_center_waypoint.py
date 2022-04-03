import os
import argparse
from file_management import load_waypoints, save_waypoints, waypoints_dir

parser = argparse.ArgumentParser(description="average two waypoints", add_help=True)
parser.add_argument("map_name", help="name of map to get waypoints from")
parser.add_argument("waypoint_1", help="waypoint_1")
parser.add_argument("waypoint_2", help="waypoint_2")
parser.add_argument("zero_theta_waypoint", help="waypoint to use for theta")
parser.add_argument("new_waypoint_name", default="center", help="name of averaged waypoint")
args = parser.parse_args()

waypoint_1 = args.waypoint_1
waypoint_2 = args.waypoint_2
zero_theta_waypoint = args.zero_theta_waypoint
map_name = args.map_name

waypoints = load_waypoints(map_name)

average_pose = (waypoints[waypoint_1] + waypoints[waypoint_2]) / 2.0
average_pose.theta = waypoints[zero_theta_waypoint].theta

waypoints[args.new_waypoint_name] = average_pose

save_waypoints(waypoints, os.path.join(waypoints_dir, map_name + ".yaml"))

print(average_pose.x)
print(average_pose.y)
print(average_pose.theta)
