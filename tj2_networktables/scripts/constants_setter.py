#!/usr/bin/python3
import math
import time
import rospy
from pprint import pprint

# from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client

from networktables import NetworkTables


class ConstantsSetter:
    def __init__(self):
        self.node_name = "constants_relay"
        rospy.init_node(
            self.node_name,
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.nt_host = rospy.get_param("~nt_host", "localhost")
        
        NetworkTables.initialize(server=self.nt_host)
        time.sleep(2.0)  # wait for table to populate
        self.base_key = "swerveLibrary"
        self.nt = NetworkTables.getTable(self.base_key)
        self.config_key = "configuration"

        self.starter_tree, self.starter_directory = self.build_tables(self.config_key)

        self.namespace = "/tj2/swerve_config"
        self.clients = self.build_clients(self.namespace, self.starter_directory)

        rospy.loginfo("%s init complete" % self.node_name)
    
    def set_all_clients(self, clients, tree):
        for nt_path, client in clients.items():
            subtable = self.recursive_get(tree, list(nt_path.split("/")))
            parameters = {}
            for key, value in subtable.items():
                if type(value) == dict:
                    continue
                parameters[key] = value
            rospy.loginfo("Reading %s. It contains %s" % (nt_path, parameters))
            client.update_configuration(parameters)

    def build_clients(self, namespace, directory):
        clients = {}
        path_mapping = self.find_dyn_paths(namespace, directory)
        for ros_path, nt_path in path_mapping.items():
            rospy.loginfo("Creating client for %s" % ros_path)
            dyn_client = Client(ros_path, timeout=10.0)
            clients[nt_path] = dyn_client
        return clients
    
    def find_dyn_paths(self, namespace, directory):
        paths = {}
        for nt_path in directory:
            split_path = nt_path.split("/")
            if split_path[0] != self.config_key:
                continue

            parent_path = "/".join(split_path[1:-1])  # ignore base key and value key
            ros_path = namespace + "/" + parent_path
            paths[ros_path] = "/".join(split_path[:-1])  # ignore value key only
        return paths
    
    def recursive_get(self, dictionary: dict, recurse_key: list):
        key = recurse_key.pop(0)
        value = dictionary[key]
        if len(recurse_key) == 0:
            return value
        elif type(value) == dict:
            return self.recursive_get(value, recurse_key)
        else:
            return value

    def build_tables(self, base_key):
        directory = {}
        tree = {base_key: {}}
        self._build_tables_recurse(self.nt, base_key, tree[base_key], directory)
        return tree, directory
    
    def _build_tables_recurse(self, table, base_key, tree, directory):
        sub_table = table.getSubTable(base_key)
        for sub_key in sub_table.getSubTables():
            full_key = base_key + "/" + sub_key
            tree[sub_key] = {}
            self._build_tables_recurse(table, full_key, tree[sub_key], directory)
        for sub_key in sub_table.getKeys():
            full_key = base_key + "/" + sub_key
            value = table.getEntry(full_key)
            tree[sub_key] = value.get()
            directory[full_key] = value.get()

    def run(self):
        print("Setting constants:")
        # pprint(self.starter_directory)
        pprint(self.starter_tree)
        self.set_all_clients(self.clients, self.starter_tree)

if __name__ == "__main__":
    node = ConstantsSetter()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
