import os
import cv2
import yaml
import rospkg

from tj2_tools.robot_state import Pose2d


rospack = rospkg.RosPack()
map_dir = os.path.join(rospack.get_path("tj2_laser_slam"), "maps")
waypoints_dir = os.path.join(rospack.get_path("tj2_waypoints"), "waypoints")


def load_map(map_name):
    map_config_filename = map_name + ".yaml"
    map_config_path = os.path.join(map_dir, map_config_filename)
    with open(map_config_path) as file:
        map_config = yaml.safe_load(file)
    map_image_filename = map_config["image"]
    if map_image_filename.startswith("/"):
        map_image_path = map_image_filename
    else:
        map_image_path = os.path.join(map_dir, map_image_filename)
    print("Loading map from %s" % map_image_path)
    map_image = cv2.imread(map_image_path)
    map_image = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)
    return (map_image, map_config)


def load_waypoints(map_name):
    waypoints_path = os.path.join(waypoints_dir, map_name + ".yaml")
    with open(waypoints_path) as file:
        waypoints_config = yaml.safe_load(file)
        if waypoints_config is None:
            waypoints_config = {}
    waypoints = {}
    for name, coords in waypoints_config.items():
        pose = Pose2d(*coords)
        waypoints[name] = pose
    return waypoints

