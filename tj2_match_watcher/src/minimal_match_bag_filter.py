#!/usr/bin/env python3

import os
import tqdm
import math
import rospy
import rospkg
from rosbag import Bag

from geometry_msgs.msg import PoseWithCovarianceStamped

from tj2_tools.robot_state import Pose2d


def transform_to_pose(transform):
    pose = Pose2d(
        transform.transform.translation.x,
        transform.transform.translation.y,
    )
    pose.theta = Pose2d.theta_from_quat(transform.transform.rotation)
    return pose


def main(directory, bag_in_name, time_start=None, time_stop=None):
    
    rospack = rospkg.RosPack()

    bag_in_path = os.path.join(rospack.get_path("tj2_match_watcher"), "bags", directory, bag_in_name)
    bag_out_path = os.path.join(
        os.path.dirname(bag_in_path),
        os.path.splitext(os.path.basename(bag_in_path))[0] + "-minimal.bag"
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
    sinistra_laser_topic = "/sinistra_laser"
    dextra_laser_topic = "/dextra_laser"
    initial_pose = None
    initial_pose_count = 0
    base_to_odom = None
    odom_to_map = None
    # center_offset = Pose2d(2.98145, 4.24575)  # center waypoint on old field map. New one has it at 0.0, 0.0
    center_offset = Pose2d(0.0, 0.0)
    try:
        with tqdm.tqdm(total=length) as pbar:
            for topic, msg, timestamp, conn_header in messages:
                pbar.update(1)
                if left_laser_topic in topic:
                    topic = "/sinistra_laser" + topic[len(left_laser_topic):]
                    msg.header.frame_id = "sinistra_laser_link"
                    bag_out.write(topic, msg, timestamp, connection_header=conn_header)
                if right_laser_topic in topic:
                    topic = "/dextra_laser" + topic[len(right_laser_topic):]
                    msg.header.frame_id = "dextra_laser_link"
                    bag_out.write(topic, msg, timestamp, connection_header=conn_header)
                if (sinistra_laser_topic in topic) or (dextra_laser_topic in topic):
                    bag_out.write(topic, msg, timestamp, connection_header=conn_header)
                if topic in ("/tj2/odom", "/tj2/hood", "/color_sensor/sensor", "/tj2/team_color"):
                    bag_out.write(topic, msg, timestamp, connection_header=conn_header)
                if topic == "/tj2/target":
                    bag_out.write("/tj2/target_record", msg, timestamp, connection_header=conn_header)
                if topic == "/initialpose":
                    bag_out.write(topic, msg, timestamp, connection_header=conn_header)
                    initial_pose_count = 10000
                # if topic.startswith("/camera/color"):
                #     bag_out.write(topic, msg, timestamp, connection_header=conn_header)
                if topic.startswith("/tj2/cargo/detections"):
                    bag_out.write("/tj2/detections", msg, timestamp, connection_header=conn_header)
                if "joint" in topic:
                    bag_out.write(topic, msg, timestamp, connection_header=conn_header)
                if topic == "/tf":
                    if timestamp - time_start < rospy.Duration(0.25):
                        continue
                    if initial_pose_count > 10:
                        continue
                    for transform in msg.transforms:
                        parent_frame = transform.header.frame_id.lstrip('/')
                        if parent_frame == "map":
                            odom_to_map = transform_to_pose(transform)
                        if parent_frame == "odom":
                            base_to_odom = transform_to_pose(transform)
                    
                    if base_to_odom is not None and odom_to_map is not None:
                        base_to_map = base_to_odom.relative_to(odom_to_map) - center_offset
                        pose = base_to_map.to_ros_pose()
                        initial_pose = PoseWithCovarianceStamped()
                        initial_pose.pose.pose = pose
                        initial_pose.pose.covariance[0] = 0.25 * 0.25
                        initial_pose.pose.covariance[7] = 0.25 * 0.25
                        initial_pose.pose.covariance[35] = math.radians(1.0)
                        initial_pose.header.frame_id = "map"
                        bag_out.write("/initialpose", initial_pose, timestamp)
                        initial_pose_count += 1

                        print(base_to_map)

                    continue

    finally:
        bag_in.close()
        bag_out.close()

if __name__ == '__main__':
    directory = "week3"
    bags = {
        # "2022_robot_2022-03-19-09-18-32.bag": 660,
        # "2022_robot_2022-03-19-10-12-13.bag": 275,
        # "2022_robot_2022-03-19-11-47-53.bag": 595,
        # "2022_robot_2022-03-19-14-03-24.bag": 290,
        # "2022_robot_2022-03-19-14-51-16.bag": 100,
        "2022_robot_2022-03-19-15-23-57.bag": 132,
        # "2022_robot_2022-03-19-15-54-11.bag": 186,
    }
    # directory = ""
    # bags = {
    #     # "2022_robot_2022-04-09-17-19-55.bag": 0
    #     # "2022_robot_2022-04-10-11-43-55.bag": 30
    #     # "2022_robot_2022-04-10-12-28-01.bag": 25
    #     # "2022_robot_2022-04-15-17-18-12.bag": 15,
    #     # "2022_robot_2022-04-10-16-59-54.bag": 0,
    #     # "2022_robot_2022-04-10-16-43-33.bag": 0,
    #     # "2022_robot_2022-04-10-16-22-54.bag": 0,
    #     # "2022_robot_2022-04-10-16-03-57.bag": 0,
    #     # "2022_robot_2022-04-21-09-33-21.bag": 0,
    #     "2022_robot_2022-04-21-15-05-39.bag": 0,
    # }
    for bag, start_time in bags.items():
        main(directory, bag, time_start=start_time)
