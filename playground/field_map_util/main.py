import os
import cv2
import yaml
import math
import rospkg
import copy
import scipy.ndimage
import numpy as np
from collections import defaultdict

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
    map_image = cv2.imread(map_image_path)
    map_image = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)
    return (map_image, map_config)


def rotate_with_warp(image, angle_radians, rotate_center, fill_value):
    rotate_matrix = cv2.getRotationMatrix2D(rotate_center, math.degrees(angle_radians), 1.0)
    rotate_I = np.eye(3)
    rotate_I[0:2] = rotate_matrix[0:2]
    xmin, ymin, xmax, ymax = 0, 0, image.shape[0], image.shape[1]
    pt1 = np.array([xmin, ymin, 1], dtype=np.int32)
    pt2 = np.array([xmax, ymin, 1], dtype=np.int32)
    pt3 = np.array([xmin, ymax, 1], dtype=np.int32)
    pt4 = np.array([xmax, ymax, 1], dtype=np.int32)
    pts = np.array([pt1, pt2, pt3, pt4])
    rotated_dims = np.dot(pts, rotate_I.T)
    rotated_dims = rotated_dims.astype(np.int32)
    new_shape = np.array([
        np.max(rotated_dims[:, 0]) - np.min(rotated_dims[:, 0]),
        np.max(rotated_dims[:, 1]) - np.min(rotated_dims[:, 1])
    ], dtype=np.int32)
    result = cv2.warpAffine(image, rotate_matrix, image.shape[1::-1], borderValue=fill_value, flags=cv2.INTER_LINEAR)
    return result

def rotate_image(image, angle_radians, fill_value):
    return scipy.ndimage.rotate(image, math.degrees(angle_radians), cval=fill_value)


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


def pose_to_pixel(pose: Pose2d, map_config, map_shape):
    origin = Pose2d(*map_config["origin"])
    pose_image_relative = pose.rotate_by(origin.theta)
    pose_image_relative = pose_image_relative.delta(origin)
    return (
        int(pose_image_relative.x / map_config["resolution"]),
        map_shape[0] - int(pose_image_relative.y / map_config["resolution"])
    )


def pixel_to_pose(xy, map_config, map_shape):
    return Pose2d(
        map_config["resolution"] * float(xy[0]) + map_config["origin"][0],
        map_config["resolution"] * (float(map_shape[0]) + map_config["origin"][1] - float(xy[1]))
    )


def pose_origin_to_pixel(origin_x, origin_y, map_resolution, map_height):
    return -origin_x / map_resolution, map_height + origin_y / map_resolution

def pixel_origin_to_pose(pixel_x, pixel_y, map_resolution, map_height):
    return -pixel_x * map_resolution, (pixel_y - map_height) * map_resolution


def pose_to_line(pose: Pose2d, map_config, map_shape, line_length):
    x0, y0 = pose_to_pixel(pose, map_config, map_shape)
    pose.theta -= map_config["origin"][2]
    y1 = int(line_length * math.sin(pose.theta)) + y0
    x1 = int(line_length * math.cos(pose.theta)) + x0
    return (x0, y0), (x1, y1)


def draw_pose(image, name, pose, map_config):
    pt1, pt2 = pose_to_line(pose, map_config, image.shape, 30)
    cv2.putText(image, name, pose_to_pixel(pose, map_config, image.shape), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (128, 0, 255))
    cv2.circle(image, pose_to_pixel(pose, map_config, image.shape), 3, (255, 0, 0), -1)
    cv2.line(image, pt1, pt2, (128, 0, 0), 2)


def draw_waypoints(map_image, map_config, waypoints):
    draw_image = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
    draw_pose(draw_image, "origin", Pose2d(theta=-map_config["origin"][2]), map_config)
    for name, pose in waypoints.items():
        draw_pose(draw_image, name, pose, map_config)
    return draw_image


def save_waypoints(waypoints, path):
    waypoints_config = {}
    for name, pose in waypoints.items():
        waypoints_config[name] = pose.to_list()

    with open(path, 'w') as file:
        yaml.dump(waypoints_config, file)

def get_image_center(image):
    return (np.array(image.shape[:2][::-1]) - 1.0) / 2.0

def draw_maps(old_map_image, new_map_image):
    cv2.imshow("old_map", old_map_image)
    cv2.imshow("new_map", new_map_image)
    while True:
        key = cv2.waitKey(-1)
        if chr(key & 0xff) == 'q':
            break

def center_map_on_pose(origin_pose: Pose2d, map_image, map_config, waypoints, new_map_name="{map_name}-centered", new_origin=(0.0, 0.0)):
    map_name = os.path.splitext(os.path.basename(map_config["image"]))[0]
    new_map_name = new_map_name.format_map(defaultdict(str, map_name=map_name))
    new_map_filename = new_map_name + ".yaml"
    new_image_filename = new_map_name + ".pgm"
    new_image_path = os.path.join(map_dir, new_image_filename)
    new_waypoints_path = os.path.join(waypoints_dir, new_map_filename)
    new_map_path = os.path.join(map_dir, new_map_filename)

    unknown_value = (map_config["occupied_thresh"] + map_config["free_thresh"]) / 2.0
    unknown_value = int(unknown_value * np.iinfo(map_image.dtype).max)

    image_rotate_angle = -origin_pose.theta
    rotated_image = rotate_image(map_image, image_rotate_angle, unknown_value)

    map_center_px = Pose2d(*get_image_center(map_image))
    rotate_center_px = Pose2d(*get_image_center(rotated_image))
    rotate_center_px.theta = -image_rotate_angle
    map_origin = Pose2d(*map_config["origin"])
    map_origin_px = Pose2d(*pose_origin_to_pixel(map_origin.x, map_origin.y, map_config["resolution"], map_image.shape[0]))

    new_map_origin_px = (map_origin_px.delta(map_center_px)).relative_to(rotate_center_px)
    new_map_origin = Pose2d(*pixel_origin_to_pose(new_map_origin_px.x, new_map_origin_px.y, map_config["resolution"], rotated_image.shape[0]))
    new_map_origin.theta = image_rotate_angle

    # map_image = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
    # rotated_image = cv2.cvtColor(rotated_image, cv2.COLOR_GRAY2BGR)

    # cv2.circle(map_image, (int(map_origin_px.x), int(map_origin_px.y)), 5, (255, 0, 0), 3)
    # cv2.circle(rotated_image, (int(new_map_origin_px.x), int(new_map_origin_px.y)), 5, (255, 0, 0), 3)
    # draw_maps(map_image, rotated_image)

    new_map_config = copy.deepcopy(map_config)
    # origin_pose.theta = 0.0
    # new_map_config["origin"] = new_map_origin.relative_to_reverse(origin_pose).to_list()
    new_map_config["origin"] = new_map_origin.to_list()
    new_map_config["image"] = new_image_filename

    # map_transform = new_map_origin.relative_to_reverse(map_origin)
    new_waypoints = {}
    for name, pose in waypoints.items():
        # new_pose = pose.relative_to_reverse(origin_pose)
        new_pose = pose
        new_pose.x += new_origin[0]
        new_pose.y += new_origin[1]
        new_waypoints[name] = new_pose
    
    with open(new_map_path, 'w') as file:
        yaml.dump(new_map_config, file)

    cv2.imwrite(new_image_path, rotated_image)
    save_waypoints(new_waypoints, new_waypoints_path)

    return rotated_image, new_map_config, new_waypoints


def center_map_on_waypoint(waypoint_name, map_image, map_config, waypoints, new_origin=(0.0, 0.0)):
    return center_map_on_pose(waypoints[waypoint_name], map_image, map_config, waypoints, f"{{map_name}}-{waypoint_name}", new_origin)

def main():
    map_name = "rapid-react-2022-02-19T07-55-53--407688-edited"
    map_image, map_config = load_map(map_name)
    waypoints = load_waypoints(map_name)

    rotated_image, new_map_config, new_waypoints = center_map_on_waypoint("center", map_image, map_config, waypoints)

    old_map_image = draw_waypoints(map_image, map_config, waypoints)
    new_map_image = draw_waypoints(rotated_image, new_map_config, new_waypoints)
    draw_maps(old_map_image, new_map_image)


if __name__ == '__main__':
    main()
