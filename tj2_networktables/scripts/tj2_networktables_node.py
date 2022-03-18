#!/usr/bin/python3
import os
import rospy
import rosnode

from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String

from networktables import NetworkTables

import pynmcli

from tj2_limelight.msg import LimelightTarget
from tj2_limelight.msg import LimelightTargetArray

from tj2_waypoints.msg import WaypointArray

from tj2_tools.launch_manager import TopicListener
from tj2_tools.robot_state import State


def get_key_recurse(tree, key, index):
    subfield = key[index]
    if index + 1 == len(key):
        return tree[subfield]
    else:
        return get_key_recurse(tree[subfield], key, index + 1)


def get_key(tree, key, default=None):
    key = key.split("/")
    try:
        return get_key_recurse(tree, key, 0)
    except KeyError:
        return default

def flatten_paths_recurse(entries, full_paths, root):
    if len(root) > 0:
        root += "/"
    for path, entry in entries.items():
        if type(entry) == dict:
            flatten_paths_recurse(entry, full_paths, root + path)
        elif type(entry) == int or type(entry) == float or type(entry) == str or type(entry) == bool:
            full_path = root + path
            full_paths[full_path] = entry
        else:
            raise ValueError("Found invalid value in entries: %s" % entry)


def flatten_paths(entries):
    full_paths = {}
    flatten_paths_recurse(entries, full_paths, "")
    return full_paths

def ros_to_nt_path(ros_path):
    if ros_path.startswith("/"):
        ros_path = ros_path[1:]
    return ros_path.replace("/", "--")

class TJ2NetworkTables:
    def __init__(self):
        self.node_name = "tj2_networktables"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown_hook)

        self.nt_host = rospy.get_param("~nt_host", "10.0.88.2")
        self.watch_topics = rospy.get_param("~watch_topics", None)
        self.watch_nodes = rospy.get_param("~watch_nodes", None)
        self.num_limelight_targets = rospy.get_param("~num_limelight_targets", 3)
        self.limelight_frame_id = rospy.get_param("~limelight_frame_id", "limelight_link")
        
        NetworkTables.initialize(server=self.nt_host)
        self.nt = NetworkTables.getTable("")

        if self.watch_topics is None:
            self.watch_topics = []
        if self.watch_nodes is None:
            self.watch_nodes = []

        self.watch_nodes_mapping = {x: ros_to_nt_path(x) for x in self.watch_nodes}
        self.watch_nodes_entries = {x: 0.0 for x in self.watch_nodes}

        self.watch_topic_mapping = {x: ros_to_nt_path(x) for x in self.watch_topics}
        self.watch_topic_entries = {x: 0.0 for x in self.watch_topics}

        self.clock_rate = rospy.Rate(2.0)  # networktable servers update at 10 Hz by default
        self.path_defaults = {
            "ROS": {
                "status": {
                    "all_nodes_ok": False,
                    "all_topics_ok": False,
                    "nodes": self.watch_nodes_entries,
                    "tunnel": {
                        "ping": 0.0
                    },
                    "recording": {
                        "status": "",
                        "bag_name": ""
                    },
                    "topics": self.watch_topic_entries,
                    "restart": False,
                    "wifi": {
                        "enable": True,
                        "status": True,
                    },
                    "waypoints": {}
                }
            },
            # "limelight": {
            #     "tv": 0.0,
            # }
        }
        # for index in range(self.num_limelight_targets):
        #     self.path_defaults["limelight"].update({
        #         "tx%d" % index: 0.0,
        #         "ty%d" % index: 0.0,
        #         "thor%d" % index: 0.0,
        #         "tvert%d" % index: 0.0,
        #     })
        self.flat_path_defaults = flatten_paths(self.path_defaults)
        self.entries = {path: self.nt.getEntry(path) for path in self.flat_path_defaults.keys()}

        self.packet_ping_sub = rospy.Subscriber("ping", Float64, self.packet_ping_callback, queue_size=10)

        self.bag_status_sub = rospy.Subscriber("bag_status", String, self.bag_status_callback, queue_size=10)
        self.bag_name_sub = rospy.Subscriber("bag_name", String, self.bag_name_callback, queue_size=10)

        self.waypoints_sub = rospy.Subscriber("waypoints", WaypointArray, self.waypoints_callback, queue_size=10)

        # self.limelight_target_pub = rospy.Publisher("/limelight/targets", LimelightTargetArray, queue_size=10)

        self.topic_listeners = {}
        for topic in self.watch_topics:
            self.topic_listeners[topic] = TopicListener(topic, 0.0)

        # self.topic_timer = rospy.Timer(rospy.Duration(2.0), self.topic_poll_callback)
        self.node_timer = rospy.Timer(rospy.Duration(2.0), self.node_poll_callback)
        # self.limelight_timer = rospy.Timer(rospy.Duration(1.0 / 10.0), self.limelight_callback)

        rospy.loginfo("%s_py init complete" % self.node_name)

    def set_entry(self, path, value):
        if path not in self.entries:
            self.entries[path] = self.nt.getEntry(path)
        return self.entries[path].setValue(value)

    def get_entry(self, path):
        default_value = self.flat_path_defaults[path]
        if type(default_value) == int:
            return int(self.entries[path].getDouble(default_value))
        elif type(default_value) == float:
            return self.entries[path].getDouble(default_value)
        elif type(default_value) == str:
            return self.entries[path].getString(default_value)
        elif type(default_value) == bool:
            return self.entries[path].getBoolean(default_value)
        else:
            raise ValueError("Invalid type for '%s': %s" % (path, default_value))

    def packet_ping_callback(self, msg):
        self.set_entry("ROS/status/tunnel/ping", msg.data)

    def topic_poll_callback(self, timer):
        all_topics_ok = True
        if len(self.watch_topics) == 0:
            all_topics_ok = False
        for topic in self.watch_topics:
            rate = self.topic_listeners[topic].get_rate(delay=0.0)
            self.set_entry("ROS/status/topics/" + ros_to_nt_path(topic), rate)
            if rate <= 0.0:
                all_topics_ok = False
        self.set_entry("ROS/status/all_topics_ok", all_topics_ok)
    
    def node_poll_callback(self, timer):
        all_nodes = list(rosnode.get_node_names())
        all_nodes_ok = True
        for node in self.watch_nodes:
            if node in all_nodes:
                all_nodes.remove(node)
                node_present = True
            else:
                all_nodes_ok = False
                node_present = False
            self.set_entry("ROS/status/nodes/" + ros_to_nt_path(node), node_present)
        
        for node in all_nodes:
            self.set_entry("ROS/status/nodes/" + ros_to_nt_path(node), True)

        self.set_entry("ROS/status/all_nodes_ok", all_nodes_ok)

    def limelight_callback(self, timer):
        has_targets = self.get_entry("limelight/tv") == 1.0
        if not has_targets:
            return
        msg = LimelightTargetArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.limelight_frame_id
        for index in range(self.num_limelight_targets):
            target_msg = LimelightTarget()
            target_msg.header = msg.header
            target_msg.tx = self.get_entry("limelight/tx%d" % index)
            target_msg.ty = self.get_entry("limelight/ty%d" % index)
            target_msg.thor = self.get_entry("limelight/thor%d" % index)
            target_msg.tvert = self.get_entry("limelight/tvert%d" % index)
            msg.targets.append(target_msg)
        self.limelight_target_pub.publish(msg)

    def bag_status_callback(self, msg):
        self.set_entry("ROS/status/recording/status", msg.data)

    def bag_name_callback(self, msg):
        self.set_entry("ROS/status/recording/bag_name", msg.data)

    def waypoints_callback(self, msg):
        for waypoint in msg.waypoints:
            name = waypoint.name
            pose = waypoint.pose

            pose_2d = State.from_ros_pose(pose)

            self.set_entry("ROS/status/waypoints/%s/x" % name, pose_2d.x)
            self.set_entry("ROS/status/waypoints/%s/y" % name, pose_2d.y)
            self.set_entry("ROS/status/waypoints/%s/theta" % name, pose_2d.theta)

    def is_wifi_enabled(self):
        results = pynmcli.get_data(self.get_wifi().execute())
        return len(results) != 0
        return False

    def disable_wifi(self):
        rospy.loginfo("Disabling wifi")
        rospy.loginfo(self.get_radio("off", root=True).execute())

    def enable_wifi(self):
        rospy.loginfo("Enabling wifi")
        rospy.loginfo(self.get_radio("on", root=True).execute())

    def get_wifi(self, *args, root=False):
        device = pynmcli.NetworkManager.Device()
        if root:
            device.command = "sudo " + device.command
        return device.wifi(*args)

    def get_radio(self, *args, root=False):
        device = pynmcli.NetworkManager.Radio()
        if root:
            device.command = "sudo " + device.command
        return device.wifi(*args)

    def run(self):
        # rospy.spin()
        rospy.sleep(2.0)  # wait for NT to populate
        self.set_entry("ROS/status/restart", False)

        prev_wifi_enable = self.is_wifi_enabled()
        self.set_entry("ROS/status/wifi/enable", prev_wifi_enable)

        while not rospy.is_shutdown():
            self.clock_rate.sleep()
            
            if self.get_entry("ROS/status/restart"):
                self.restart_roslaunch()
            
            self.set_entry("ROS/status/wifi/status", self.is_wifi_enabled())
            enable_wifi = self.get_entry("ROS/status/wifi/enable")
            if enable_wifi != prev_wifi_enable:
                if enable_wifi:
                    self.enable_wifi()
                else:
                    self.disable_wifi()
                prev_wifi_enable = enable_wifi

    def restart_roslaunch(self):
        rospy.logwarn("RESTARTING ROSLAUNCH FROM NETWORK TABLES")
        os.system("sudo systemctl restart roslaunch.service")
        while not rospy.is_shutdown():
            pass

    def shutdown_hook(self):
        all_nodes = list(rosnode.get_node_names())
        for node in self.watch_nodes:
            self.set_entry("ROS/status/nodes/" + ros_to_nt_path(node), False)
        for node in all_nodes:
            self.set_entry("ROS/status/nodes/" + ros_to_nt_path(node), False)
        self.set_entry("ROS/status/all_nodes_ok", False)
        for topic in self.watch_topics:
            self.set_entry("ROS/status/topics/" + ros_to_nt_path(topic), 0.0)
        self.set_entry("ROS/status/all_topics_ok", False)

if __name__ == "__main__":
    node = TJ2NetworkTables()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
