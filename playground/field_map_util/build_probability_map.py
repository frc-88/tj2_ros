import os
import cv2
import csv
import sys
import math
import rospkg
import argparse
import numpy as np

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.robot_state import Pose2d
from tj2_tools.occupancy_grid import OccupancyGridManager

from file_management import load_waypoints


rospack = rospkg.RosPack()


def draw_field_shape(ogm, field_pts, probability, robot_radius_px, is_closed, is_filled):
    polylines_pts = []
    for point in field_pts:
        img_x1, img_y1 = ogm.get_costmap_x_y(point.x, point.y)
        polylines_pts.append([img_x1, img_y1])
    polylines_pts = np.array(polylines_pts, np.int32)

    probability = 1.0 - probability

    cv2.polylines(ogm.grid_data, [polylines_pts], is_closed, (probability * 100,), robot_radius_px)
    if is_filled:
        cv2.fillPoly(ogm.grid_data, [polylines_pts], (probability * 100,))


def build_map(practice_map_name, field_map_name, data_path, lines=None, hood_state=None, robot_radius=0.5):
    turret_base_link_x_offset = -0.050456

    # in_map_path = os.path.join(rospack.get_path("tj2_laser_slam"), "maps", practice_map_name + ".yaml")
    field_map_path = os.path.join(rospack.get_path("tj2_laser_slam"), "maps", field_map_name + ".yaml")
    field_map_name = os.path.splitext(os.path.basename(field_map_path))[0]
    hood_suffix = "-%s" % hood_state if hood_state is not None else ""
    out_map_path = os.path.join(rospack.get_path("tj2_target"), "maps", field_map_name + hood_suffix + ".yaml")

    ogm = OccupancyGridManager.from_cost_file(field_map_path)
    practice_waypoints = load_waypoints(practice_map_name)
    practice_center = practice_waypoints["center"]

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
                ),
                "hood": row[header.index("hood")]
            }
            if hood_state is not None and data["hood"] != hood_state:
                continue
            print("%s\t%0.2f\t%0.3f" % (data["hood"], data["probability"], data["distance"]))
            probabilities.append(data)

    field_map = ogm.get_image()

    ogm.grid_data[:] = 100
    probabilities.sort(key=lambda x: x["distance"], reverse=True)

    robot_radius_px = int(robot_radius / ogm.resolution)

    for row in probabilities:
        # calculate field position using distance
        # distance = row["distance"]
        # x = field_center.x + distance + turret_base_link_x_offset
        # y = field_center.y + turret_base_link_x_offset
        
        # calculate field position using robot pose
        field_pose = row["pose"].relative_to_reverse(practice_center).relative_to(field_center)
        x = field_pose.x
        y = field_pose.y
        image_x, image_y = ogm.get_costmap_x_y(x, y)
        
        origin = ogm.get_costmap_x_y(field_center.x, field_center.y)

        x = image_x - origin[0]
        y = image_y - origin[1]
        radius = int(math.sqrt(x * x + y * y))
        radius += robot_radius_px
        if radius < 0:
            radius = 1
        probability = 1.0 - row["probability"]
        
        cv2.circle(ogm.grid_data, origin, radius, (probability * 100,), -1)
        # cv2.circle(ogm.grid_data, (image_x, image_y), robot_radius_px, (probability * 100,), -1)

    field_center_rotate = Pose2d.from_state(field_center)
    field_center_rotate.theta += math.pi
    
    for row in lines:
        field_pts = []
        practice_pts = []
        is_closed = row.get("is_closed", True)
        filled = row.get("filled", True)
        use_field = row.get("use_field", False)
        radius = int(row.get("radius", robot_radius) / ogm.resolution)
        for waypoint in row["pts"]:
            if use_field:
                field_pt = point = field_waypoints[waypoint]
            else:
                point = practice_waypoints[waypoint]
                field_pt = point.relative_to_reverse(practice_center).relative_to(field_center)
            field_pts.append(field_pt)
            practice_pts.append(point)

        draw_field_shape(ogm, field_pts, row["probability"], radius, is_closed, filled)

        if row.get("mirror", False):
            mirror_field_pts = []
            for index in range(len(field_pts)):
                if use_field:
                    field_pt = field_pts[index]
                    mirror_field_pt = field_pt.relative_to(field_center_rotate)
                else:
                    practice_pt = practice_pts[index]
                    mirror_field_pt = practice_pt.relative_to_reverse(practice_center).relative_to(field_center_rotate)
                mirror_field_pts.append(mirror_field_pt)

            draw_field_shape(ogm, mirror_field_pts, row["probability"], radius, is_closed, filled)

    print("Writing to %s" % out_map_path)
    ogm.to_file(out_map_path)

    results = {
        "field_map": field_map,
        "practice_center": practice_center,
        "field_center": field_center,
        "ogm": ogm,
    }

    return results


def mouse_callback(event, x, y, flags, param):
    if (event == cv2.EVENT_LBUTTONDOWN):
        mode = param["mode"]
        results = param[mode]
        ogm = results["ogm"]
        practice_center = results["practice_center"]
        field_center = results["field_center"]
        field_pose = Pose2d(*ogm.get_world_x_y(x, y))
        practice_pose = field_pose.relative_to_reverse(field_center).relative_to(practice_center)

        cost = ogm.get_cost_from_costmap_x_y(x, y)
        probability = 1.0 - (cost / 100.0)

        print("mode:", mode)
        print("practice:", practice_pose.to_list())
        print("field:", field_pose.to_list())
        print("pixel:", x, y)
        print("probability:", probability)


def main():
    parser = argparse.ArgumentParser(description="build probability map", add_help=True)
    parser.add_argument("practice_map_name", help="name of map recorded_data.csv was taken on")
    parser.add_argument("field_map_name", help="name of map to generate probability map for")
    parser.add_argument("-d", "--data", default="recorded_data.csv", help="name of csv to generate probability distributions from")
    parser.add_argument("-s", "--show", action="store_true", help="Show image")
    args = parser.parse_args()

    # practice_map_name = "br-114-03-26-2022"
    # field_map_name = "br-114-03-26-2022"
    # field_map_name = "br-114-03-29-2022"
    # field_map_name = "rapid-react-2022-02-19T07-55-53--407688-edited"

    lines = []
    hangar_radius = 0.2
    lines.append({"pts": ["hanger_se", "hanger_sw"], "probability": 0.0, "mirror": True, "use_field": True, "radius": hangar_radius})
    lines.append({"pts": ["hanger_ne", "hanger_nw"], "probability": 0.0, "mirror": True, "use_field": True, "radius": hangar_radius})

    bar_width = 0.3
    lines.append({"pts": ["lower_bar_n", "lower_bar_s"], "probability": 0.0, "mirror": True, "use_field": True, "radius": bar_width})
    lines.append({"pts": ["mid_bar_n", "mid_bar_s"], "probability": 0.0, "mirror": True, "use_field": True, "radius": bar_width})
    lines.append({"pts": ["high_bar_n", "high_bar_s"], "probability": 0.0, "mirror": True, "use_field": True, "radius": bar_width})
    lines.append({"pts": ["trav_bar_n", "trav_bar_s"], "probability": 0.0, "mirror": True, "use_field": True, "radius": bar_width})

    hub_radius = 0.2
    lines.append({"pts": ["hub_pt1", "hub_pt2", "hub_pt3", "hub_pt4", "hub_pt5", "hub_pt6"], "probability": 0.0, "mirror": True, "use_field": True, "radius": hub_radius})
    lines.append({"pts": ["hub_pt1_r", "hub_pt2_r", "hub_pt3_r", "hub_pt4_r", "hub_pt5_r", "hub_pt6_r"], "probability": 0.0, "mirror": True, "use_field": True, "radius": hub_radius})

    data_path = os.path.join(rospack.get_path("tj2_target"), "config", args.data)
    results_up = build_map(args.practice_map_name, args.field_map_name, data_path, lines, hood_state="up")
    results_down = build_map(args.practice_map_name, args.field_map_name, data_path, lines, hood_state="down")
    results = {
        "mode": "up",
        "up": results_up,
        "down": results_down,
    }

    if args.show:
        window_name = "map"
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, mouse_callback, param=results)

        show_image_up = cv2.addWeighted(results_up["field_map"], 0.5, results_up["ogm"].get_image(), 0.5, 0.0)
        show_image_down = cv2.addWeighted(results_up["field_map"], 0.5, results_down["ogm"].get_image(), 0.5, 0.0)

        cv2.imshow(window_name, show_image_up)
        while True:
            key = chr(cv2.waitKey(-1) & 0xff)
            if key == 'u':
                cv2.imshow(window_name, show_image_up)
                results["mode"] = "up"
            elif key == 'd':
                cv2.imshow(window_name, show_image_down)
                results["mode"] = "down"

            if key == 'q':
                break

if __name__ == "__main__":
    main()
