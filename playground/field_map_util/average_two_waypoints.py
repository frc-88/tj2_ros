import yaml
import argparse
from file_management import load_waypoints

parser = argparse.ArgumentParser(description="average two waypoints", add_help=True)
parser.add_argument("map_name", help="name of map to get waypoints from")
parser.add_argument("waypoint_1", help="waypoint_1")
parser.add_argument("waypoint_2", help="waypoint_2")
args = parser.parse_args()

waypoint_1 = args.waypoint_1
waypoint_2 = args.waypoint_2
map_name = args.map_name

waypoints = load_waypoints(map_name)

average_pose = (waypoints[waypoint_1] + waypoints[waypoint_2]) / 2.0

print(average_pose.x)
print(average_pose.y)
print(average_pose.theta)