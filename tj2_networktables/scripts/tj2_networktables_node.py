#!/usr/bin/python3
import rospy
import rosnode

from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String

from networktables import NetworkTables

from tj2_tools.launch_manager import TopicListener


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
    for path, entry in entries.items():
        if type(entry) == dict:
            flatten_paths_recurse(entry, full_paths, root + "/" + path)
        elif type(entry) == int or type(entry) == float or type(entry) == str or type(entry) == bool:
            full_path = root + "/" + path
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

        self.clock_rate = rospy.Rate(10.0)  # networktable servers update at 10 Hz by default
        self.path_defaults = {
            "ROS": {
                "status": {
                    "all_nodes_ok": False,
                    "all_topics_ok": False,
                    "nodes": self.watch_nodes_entries,
                    "tunnel": {
                        "rate": 0.0,
                        "count": 0,
                        "ping": 0.0
                    },
                    "recording": {
                        "status": "",
                        "bag_name": ""
                    },
                    "topics": self.watch_topic_entries
                }
            }
        }
        self.flat_path_defaults = flatten_paths(self.path_defaults)
        self.entries = {path: self.nt.getEntry(path) for path in self.flat_path_defaults.keys()}

        self.packet_count_sub = rospy.Subscriber("packet_count", Int32, self.packet_count_callback, queue_size=10)
        self.packet_rate_sub = rospy.Subscriber("packet_rate", Float64, self.packet_rate_callback, queue_size=10)
        self.packet_ping_sub = rospy.Subscriber("ping", Float64, self.packet_ping_callback, queue_size=10)

        self.bag_status_sub = rospy.Subscriber("bag_status", String, self.bag_status_callback, queue_size=10)
        self.bag_name_sub = rospy.Subscriber("bag_name", String, self.bag_name_callback, queue_size=10)

        self.topic_listeners = {}
        for topic in self.watch_topics:
            self.topic_listeners[topic] = TopicListener(topic, 0.0)

        self.topic_timer = rospy.Timer(rospy.Duration(3.0), self.topic_poll_callback)
        self.node_timer = rospy.Timer(rospy.Duration(3.0), self.node_poll_callback)

        rospy.loginfo("%s init complete" % self.node_name)

    def set_entry(self, path, value):
        if path in self.entries:
            return self.entries[path].setValue(value)
        else:
            return self.nt.getEntry(path).setValue(value)

    def get_entry(self, path):
        default_value = self.flat_path_defaults[path]
        if type(default_value) == int:
            return int(self.entries[path].getDouble(default_value))
        elif type(default_value) == float:
            return self.entries[path].getDouble(default_value)
        elif type(default_value) == str:
            return self.entries[path].getString(default_value)
        else:
            raise ValueError("Invalid type for '%s': %s" % (path, default_value))

    def packet_count_callback(self, msg):
        self.set_entry("/ROS/status/tunnel/count", msg.data)
    
    def packet_rate_callback(self, msg):
        self.set_entry("/ROS/status/tunnel/rate", msg.data)

    def packet_ping_callback(self, msg):
        self.set_entry("/ROS/status/tunnel/ping", msg.data)

    def topic_poll_callback(self, timer):
        all_topics_ok = True
        if len(self.watch_topics) == 0:
            all_topics_ok = False
        for topic in self.watch_topics:
            rate = self.topic_listeners[topic].get_rate(delay=0.0)
            self.set_entry("/ROS/status/topics/" + ros_to_nt_path(topic), rate)
            if rate <= 0.0:
                all_topics_ok = False
        self.set_entry("/ROS/status/all_topics_ok", all_topics_ok)
    
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
            self.set_entry("/ROS/status/nodes/" + ros_to_nt_path(node), node_present)
        
        for node in all_nodes:
            self.set_entry("/ROS/status/nodes/" + ros_to_nt_path(node), True)

        self.set_entry("/ROS/status/all_nodes_ok", all_nodes_ok)

    def bag_status_callback(self, msg):
        self.set_entry("/ROS/recording/status", msg.data)

    def bag_name_callback(self, msg):
        self.set_entry("/ROS/recording/bag_name", msg.data)


    def run(self):
        # rospy.spin()
        rospy.sleep(2.0)  # wait for NT to populate
        while not rospy.is_shutdown():
            self.clock_rate.sleep()
            self.topic_poll_callback(None)
            self.node_poll_callback(None)

    def shutdown_hook(self):
        all_nodes = list(rosnode.get_node_names())
        for node in self.watch_nodes:
            self.set_entry("/ROS/status/nodes/" + ros_to_nt_path(node), False)
        for node in all_nodes:
            self.set_entry("/ROS/status/nodes/" + ros_to_nt_path(node), False)
        self.set_entry("/ROS/status/all_nodes_ok", False)
        for topic in self.watch_topics:
            self.set_entry("/ROS/status/topics/" + ros_to_nt_path(topic), 0.0)
        self.set_entry("/ROS/status/all_topics_ok", False)

if __name__ == "__main__":
    node = TJ2NetworkTables()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
