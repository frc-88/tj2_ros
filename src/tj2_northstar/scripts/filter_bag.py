import os
import tqdm
import argparse
from rosbag import Bag
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser("calibrate")
    parser.add_argument("bag", type=str)
    args = parser.parse_args()

    in_bag_path = args.bag
    in_bag = Bag(in_bag_path)

    out_dir = os.path.dirname(in_bag_path)
    in_name = os.path.splitext(os.path.basename(in_bag_path))[0]
    out_bag_path = os.path.join(out_dir, in_name + "-filtered.bag")
    out_bag = Bag(out_bag_path, "w")

    messages = in_bag.read_messages()
    length = in_bag.get_message_count()
    pbar = tqdm.tqdm(total=length)
    for topic, msg, timestamp in messages:
        pbar.update(1)
        if topic == "/tf":
            new_transforms = TFMessage()
            for transform_msg in msg.transforms:
                parent = transform_msg.header.frame_id
                child = transform_msg.child_frame_id
                if parent == "odom" and child == "base_link":
                    new_transforms.transforms.append(transform_msg)
            if len(new_transforms.transforms) == 0:
                continue
            out_bag.write(topic, new_transforms, timestamp)
        else:
            out_bag.write(topic, msg, timestamp)
    in_bag.close()
    out_bag.close()


if __name__ == "__main__":
    main()
