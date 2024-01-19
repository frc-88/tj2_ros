#!/usr/bin/env python3
import argparse

import rosbag
import tqdm


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("in_bag")
    parser.add_argument("out_bag", nargs="?")
    args = parser.parse_args()

    excluded_frames = ("map", "odom")

    in_path = args.in_bag
    out_path = args.out_bag

    if not out_path:
        out_path = in_path.replace(".bag", "_filtered.bag")
    print(f"Filtering {in_path} to {out_path}")

    in_bag = rosbag.Bag(in_path)
    out_bag = rosbag.Bag(out_path, "w")
    bag_length = in_bag.get_message_count()
    pbar = tqdm.tqdm(total=bag_length)
    for topic, msg, t in in_bag.read_messages():
        pbar.update()
        if "tag_detections" in topic:
            out_bag.write(topic, msg, t)
        elif topic == "/clock":
            continue
        elif topic == "/tf":
            filtered_transform = []
            for transform in msg.transforms:
                if transform.header.frame_id not in excluded_frames:
                    filtered_transform.append(transform)
            if len(filtered_transform) == 0:
                continue
            msg.transforms = filtered_transform
            out_bag.write(topic, msg, t)
        else:
            out_bag.write(topic, msg, t)
    out_bag.close()
    in_bag.close()


if __name__ == "__main__":
    main()
