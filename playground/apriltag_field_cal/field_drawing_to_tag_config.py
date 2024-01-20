import os

import yaml
from geometry_msgs.msg import Quaternion, Vector3
from tj2_tools.transforms.rpy import RPY
from tj2_tools.transforms.transform3d import Transform3D


def in_to_m(inches: float) -> float:
    return inches * 0.0254


def main() -> None:
    local_dir = os.path.dirname(os.path.realpath(__file__))
    with open(os.path.join(local_dir, "drawing.yaml"), "r") as f:
        drawing = yaml.safe_load(f)

    optical_to_field_tf = Transform3D.from_position_and_rpy(
        Vector3(0, 0, 0),
        RPY.from_degrees((-90.0, -90.0, 0.0)),
    ).inverse()
    out_config = []
    comments = {}
    for tag_config in drawing:
        tag_id = tag_config["id"]
        tag_transform = Transform3D.from_position_and_rpy(
            Vector3(in_to_m(tag_config["x"]), in_to_m(tag_config["y"]), in_to_m(tag_config["z"])),
            RPY.from_degrees((tag_config["roll"], tag_config["pitch"], tag_config["yaw"])),
        )
        rotated_in_place = tag_transform.rotation_matrix @ optical_to_field_tf.rotation_matrix
        tag_transform = Transform3D.from_position_and_quaternion(tag_transform.position, Quaternion(w=1.0))
        tag_transform.tfmat[0:3, 0:3] = rotated_in_place
        tag_size = in_to_m(tag_config["size"])
        out_config.append(
            {
                "id": tag_id,
                "x": round(tag_transform.x, 4),
                "y": round(tag_transform.y, 4),
                "z": round(tag_transform.z, 4),
                "qx": round(tag_transform.quaternion.x, 4),
                "qy": round(tag_transform.quaternion.y, 4),
                "qz": round(tag_transform.quaternion.z, 4),
                "qw": round(tag_transform.quaternion.w, 4),
                "size": tag_size,
            }
        )
        comments[tag_id] = tag_config["comment"]

    element_widths = {key: [] for key in out_config[0]}
    for tag_config in out_config:
        for key in tag_config:
            element_widths[key].append(len(str(tag_config[key])))
    max_widths = {key: max(element_widths[key]) for key in element_widths}
    for tag_config in out_config:
        out_row = "{"
        for index, key in enumerate(tag_config):
            spaced_string = str(tag_config[key]).rjust(max_widths[key])
            out_row += f"{key}: {spaced_string}"
            if index != len(tag_config) - 1:
                out_row += ", "
        out_row += "},"
        out_row += f"  # {comments[tag_config['id']]}"
        print(out_row)


if __name__ == "__main__":
    main()
