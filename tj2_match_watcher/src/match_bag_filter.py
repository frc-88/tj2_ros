#!/usr/bin/env python3

import os
import tqdm
import rospy
import rospkg
from rosbag import Bag


def main(directory, bag_in_name, time_start=None, time_stop=None):
    rospack = rospkg.RosPack()

    bag_in_path = os.path.join(rospack.get_path("tj2_match_watcher"), "bags", directory, bag_in_name)
    bag_out_path = os.path.join(
        os.path.dirname(bag_in_path),
        os.path.splitext(os.path.basename(bag_in_path))[0] + "-filter.bag"
    )
    
    bag_in = Bag(bag_in_path)
    bag_out = Bag(bag_out_path, 'w')

    if time_start is not None:
        time_start = rospy.Time(bag_in.get_start_time() + time_start)
    if time_stop is not None:
        time_stop = rospy.Time(bag_in.get_start_time() + time_stop)
    

    messages = bag_in.read_messages(
        start_time=time_start,
        end_time=time_stop,
        return_connection_header=True)
    length = bag_in.get_message_count()

    left_laser_topic = "/left_laser"
    right_laser_topic = "/right_laser"
    camera_timer = None
    try:
        with tqdm.tqdm(total=length) as pbar:
            for topic, msg, timestamp, conn_header in messages:
                pbar.update(1)
                if left_laser_topic in topic:
                    topic = "/sinistra_laser" + topic[len(left_laser_topic):]
                    msg.header.frame_id = "sinistra_laser_link"
                if right_laser_topic in topic:
                    topic = "/dextra_laser" + topic[len(right_laser_topic):]
                    msg.header.frame_id = "dextra_laser_link"
                if topic.startswith("/camera"):
                    if topic != "/camera/color/image_raw":
                        continue
                    if camera_timer is None:
                        camera_timer = timestamp
                        continue
                    rate = 1.0 / (timestamp - camera_timer).to_sec()
                    if rate > 15.0:
                        continue
                    camera_timer = timestamp
                if topic.startswith("/move_base"):
                    continue
                if topic.startswith("/pursuit"):
                    continue
                if topic == "/tf":
                    should_skip = True
                    for transform in msg.transforms:
                        parent_frame = transform.header.frame_id.lstrip('/')
                        # child_frame = transform.child_frame_id.lstrip('/')
                        # print("%s -> %s" % (parent_frame, child_frame))
                        if parent_frame == "map" or parent_frame == "odom":
                            should_skip = False
                    if should_skip:
                        continue

                bag_out.write(topic, msg, timestamp, connection_header=conn_header)
    finally:
        bag_in.close()
        bag_out.close()

if __name__ == '__main__':
    directory = "week3"
    
    bags = {
        # "2022_robot_2022-03-19-10-12-13.bag": 275,
        # "2022_robot_2022-03-19-11-47-53.bag": 595,
        # "2022_robot_2022-03-19-14-03-24.bag": 290,
        # "2022_robot_2022-03-19-14-51-16.bag": 100,
        # "2022_robot_2022-03-19-15-23-57.bag": 132,
        "2022_robot_2022-03-19-15-54-11.bag": 186,
    }
    for bag, start_time in bags.items():
        main(directory, bag, time_start=start_time)
