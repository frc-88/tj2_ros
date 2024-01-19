import json
import math

import numpy as np
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt
from rosbag import Bag
from sklearn.cluster import KMeans
from tj2_tools.transforms.rpy import RPY
from tj2_tools.transforms.transform3d import Transform3D


def load_bag(filename: str) -> Bag:
    return Bag("/media/storage/bags/" + filename, "r")


def main() -> None:
    tags = []
    bag = load_bag("fieldcal.bag")
    base_to_optical_tf = Transform3D.from_position_and_rpy(
        Vector3(0, 0, 0), RPY((math.radians(-90), math.radians(90), 0))
    )

    for topic, msg, timestamp in bag.read_messages():
        if topic == "/northstar/filtered_detections":
            if len(msg.detections) > 0:
                tags.append(msg)

    global_poses: dict[str, list[Transform3D]] = {}
    sizes: dict[str, float] = {}
    for tag in tags:
        tag: AprilTagDetectionArray
        for detection in tag.detections:
            detection: AprilTagDetection
            if len(detection.id) != 1:
                continue
            tag_id = str(detection.id[0])
            sizes[tag_id] = detection.size[0]
            if tag_id not in global_poses:
                global_poses[tag_id] = []
            detection_tf = Transform3D.from_pose_msg(detection.pose.pose.pose)
            global_poses[tag_id].append(detection_tf)

    map_tags: dict[str, Transform3D] = {}
    for tag_id, poses in global_poses.items():
        data = np.array([np.append(pose.position_array, pose.rpy.to_array()) for pose in poses])
        kmeans = KMeans(n_clusters=3, n_init=10)
        kmeans.fit(data)

        tag_transform = Transform3D.from_position_and_rpy(
            Vector3(*kmeans.cluster_centers_[0, 0:3].tolist()), RPY(kmeans.cluster_centers_[0, 3:6].tolist())
        )
        tag_rotation = tag_transform.rotation_matrix @ base_to_optical_tf.rotation_matrix
        optical_tag_transform = np.eye(4)
        optical_tag_transform[0:3, 0:3] = tag_rotation
        optical_tag_transform[0:3, 3] = tag_transform.position_array
        map_tags[tag_id] = Transform3D(optical_tag_transform)

        x = data[:, 0]
        y = data[:, 1]
        line = plt.plot(x, y, label=tag_id, marker=".", linestyle="none", alpha=0.2)
        plt.plot(
            kmeans.cluster_centers_[0, 0],
            kmeans.cluster_centers_[0, 1],
            color=line[0].get_color(),
            marker="x",
            markersize=10,
        )
    for tag_id, pose in map_tags.items():
        tag_data = {
            "id": int(tag_id),
            "size": sizes[tag_id],
            "x": round(pose.x, 4),
            "y": round(pose.y, 4),
            "z": round(pose.z, 4),
            "qx": round(pose.quaternion.x, 4),
            "qy": round(pose.quaternion.y, 4),
            "qz": round(pose.quaternion.z, 4),
            "qw": round(pose.quaternion.w, 4),
        }
        print(json.dumps(tag_data))

    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
