import json
import math

import numpy as np
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from rosbag import Bag
from scipy import stats
from sklearn.cluster import KMeans
from tj2_tools.transforms.rpy import RPY
from tj2_tools.transforms.transform3d import Transform3D


def load_bag(filename: str) -> Bag:
    return Bag("/media/storage/bags/" + filename, "r")


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


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

    global_poses: dict[int, list[Transform3D]] = {}
    sizes: dict[int, float] = {}
    for tag in tags:
        tag: AprilTagDetectionArray
        for detection in tag.detections:
            detection: AprilTagDetection
            if len(detection.id) != 1:
                continue
            tag_id = int(detection.id[0])
            sizes[tag_id] = detection.size[0]
            if tag_id not in global_poses:
                global_poses[tag_id] = []
            detection_tf = Transform3D.from_pose_msg(detection.pose.pose.pose)
            global_poses[tag_id].append(detection_tf)

    fig = plt.figure(figsize=(15, 15))
    axis = fig.add_subplot(111, projection="3d")
    arrow_length = 0.5

    map_tags: dict[int, Transform3D] = {}
    for tag_id, poses in global_poses.items():
        data = np.array([np.append(pose.position_array, pose.rpy.to_array()) for pose in poses])

        kmeans = KMeans(n_clusters=3, n_init="auto")
        kmeans.fit(data)
        biggest_cluster = stats.mode(kmeans.labels_).mode[0]
        average_position = kmeans.cluster_centers_[biggest_cluster, 0:3]
        average_rotation = kmeans.cluster_centers_[biggest_cluster, 3:6]

        # average_position = np.mean(data[:, 0:3], axis=0)
        # average_rotation = np.mean(data[:, 3:6], axis=0)

        tag_transform = Transform3D.from_position_and_rpy(
            Vector3(*average_position.tolist()), RPY(average_rotation.tolist())
        )

        map_tags[tag_id] = tag_transform
        # tag_rotation = tag_transform.rotation_matrix @ base_to_optical_tf.rotation_matrix
        # optical_tag_transform = np.eye(4)
        # optical_tag_transform[0:3, 0:3] = tag_rotation
        # optical_tag_transform[0:3, 3] = tag_transform.position_array
        # map_tags[tag_id] = Transform3D(optical_tag_transform)

        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]
        line = axis.plot(x, y, z, label=tag_id, marker=".", linestyle="none", alpha=0.2)
        # for pose in poses:
        #     normal = pose.rotation_matrix @ np.array([0, 0, arrow_length])
        #     next_position = pose.position_array + normal
        #     arrow = Arrow3D(
        #         [pose.position_array[0], next_position[0]],
        #         [pose.position_array[1], next_position[1]],
        #         [pose.position_array[2], next_position[2]],
        #         mutation_scale=20,
        #         lw=3,
        #         arrowstyle="-|>",
        #         color=line[0].get_color(),
        #     )
        #     axis.add_artist(arrow)

        normal = tag_transform.rotation_matrix @ np.array([0, 0, 1])
        normal /= np.linalg.norm(normal)
        normal *= arrow_length
        next_position = tag_transform.position_array + normal
        arrow = Arrow3D(
            [average_position[0], next_position[0]],
            [average_position[1], next_position[1]],
            [average_position[2], next_position[2]],
            mutation_scale=20,
            lw=3,
            arrowstyle="-|>",
            color=line[0].get_color(),
        )
        axis.add_artist(arrow)

    tag_ids = sorted(map_tags.keys())
    for tag_id in tag_ids:
        pose = map_tags[tag_id]
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
