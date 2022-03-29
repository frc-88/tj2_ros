import os
import cv2
import csv
import sys
import math
import rospkg
import numpy as np

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.robot_state import Pose2d
from tj2_tools.occupancy_grid import OccupancyGridManager

from file_management import load_waypoints


def draw_field_line(ogm, field_pt1, field_pt2, probability, robot_radius_px):
    img_x1, img_y1 = ogm.get_costmap_x_y(field_pt1.x, field_pt1.y)
    img_x2, img_y2 = ogm.get_costmap_x_y(field_pt2.x, field_pt2.y)

    probability = 1.0 - probability

    cv2.line(ogm.grid_data, (img_x1, img_y1), (img_x2, img_y2), (probability * 100,), robot_radius_px)


def build_map(practice_map_name, field_map_name, lines=None):
    turret_base_link_x_offset = -0.050456

    rospack = rospkg.RosPack()
    # in_map_path = os.path.join(rospack.get_path("tj2_laser_slam"), "maps", practice_map_name + ".yaml")
    field_map_path = os.path.join(rospack.get_path("tj2_laser_slam"), "maps", field_map_name + ".yaml")
    out_map_path = os.path.join(rospack.get_path("tj2_turret"), "maps", field_map_path + ".yaml")
    data_path = os.path.join(rospack.get_path("tj2_turret"), "config", "recorded_data.csv")

    ogm = OccupancyGridManager.from_cost_file(field_map_path)
    waypoints = load_waypoints(practice_map_name)
    center = waypoints["center"]

    field_waypoints = load_waypoints(field_map_name)
    field_center = field_waypoints["center"]

    if lines is None:
        lines = []

    probabilities = []
    with open(data_path) as file:
        reader = csv.reader(file)
        header = next(reader)
        for row in reader:
            if row[0].strip().startswith("#"):
                continue
            if row[0] != "prob":
                continue
            data = {
                "probability": float(row[header.index("value")]),
                "distance": float(row[header.index("distance")]),
                "pose": Pose2d(
                    float(row[header.index("x")]),
                    float(row[header.index("y")]),
                    float(row[header.index("theta")]),
                )
            }
            hood = row[header.index("hood")]
            # if hood == "up":
            #     continue
            print("%s\t%0.2f\t%0.3f" % (hood, data["probability"], data["distance"]))
            probabilities.append(data)

    field_map = ogm.get_image()

    ogm.grid_data[:] = 100
    probabilities.sort(key=lambda x: x["distance"], reverse=True)

    robot_radius = int(0.5 / ogm.resolution)

    for row in probabilities:
        # calculate field position using distance
        # distance = row["distance"]
        # x = field_center.x + distance + turret_base_link_x_offset
        # y = field_center.y + turret_base_link_x_offset
        
        # calculate field position using robot pose
        field_pose = row["pose"].relative_to_reverse(center).relative_to(field_center)
        x = field_pose.x
        y = field_pose.y
        image_x, image_y = ogm.get_costmap_x_y(x, y)
        
        origin = ogm.get_costmap_x_y(field_center.x, field_center.y)

        x = image_x - origin[0]
        y = image_y - origin[1]
        radius = int(math.sqrt(x * x + y * y))
        radius += robot_radius
        if radius < 0:
            radius = 1
        probability = 1.0 - row["probability"]
        
        cv2.circle(ogm.grid_data, origin, radius, (probability * 100,), -1)
        # cv2.circle(ogm.grid_data, (image_x, image_y), int(robot_radius), (probability * 100,), -1)

    field_center_rotate = Pose2d.from_state(field_center)
    field_center_rotate.theta += math.pi
    for row in lines:
        pt1 = waypoints[row["pt1"]]
        pt2 = waypoints[row["pt2"]]
        field_pt1 = pt1.relative_to_reverse(center).relative_to(field_center)
        field_pt2 = pt2.relative_to_reverse(center).relative_to(field_center)

        draw_field_line(ogm, field_pt1, field_pt2, row["probability"], robot_radius)

        if row.get("mirror", False):
            mirror_field_pt1 = pt1.relative_to_reverse(center).relative_to(field_center_rotate)
            mirror_field_pt2 = pt2.relative_to_reverse(center).relative_to(field_center_rotate)
            draw_field_line(ogm, mirror_field_pt1, mirror_field_pt2, row["probability"], robot_radius)

    show_image = cv2.addWeighted(field_map, 0.5, ogm.get_image(), 0.5, 0.0)

    ogm.to_file(out_map_path)

    results = {
        "show_image": show_image,
        "practice_center": center,
        "field_center": field_center,
        "ogm": ogm,
    }

    return results


def mouse_callback(event, x, y, flags, param):
    if (event == cv2.EVENT_LBUTTONDOWN):
        ogm = param["ogm"]
        practice_center = param["practice_center"]
        field_center = param["field_center"]
        field_pose = Pose2d(*ogm.get_world_x_y(x, y))
        practice_pose = field_pose.relative_to_reverse(field_center).relative_to(practice_center)

        print("practice:", practice_pose.to_list())
        print("field:", field_pose.to_list())
        print("pixel:", x, y)


def main():
    practice_map_name = "br-114-03-26-2022"
    # field_map_name = "br-114-03-26-2022"
    field_map_name = "rapid-react-2022-02-19T07-55-53--407688-edited"

    lines = []
    lines.append({"pt1": "lower_bar_1", "pt2": "lower_bar_2", "probability": 0.0, "mirror": True})
    lines.append({"pt1": "mid_bar_1", "pt2": "mid_bar_2", "probability": 0.0, "mirror": True})
    lines.append({"pt1": "high_bar_1", "pt2": "high_bar_2", "probability": 0.0, "mirror": True})
    lines.append({"pt1": "trav_bar_1", "pt2": "trav_bar_2", "probability": 0.0, "mirror": True})

    results = build_map(practice_map_name, field_map_name, lines)

    window_name = "map"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback, param=results)
    cv2.imshow(window_name, results["show_image"])
    while True:
        key = chr(cv2.waitKey(-1) & 0xff)
        if key == 'q':
            break

if __name__ == "__main__":
    main()
