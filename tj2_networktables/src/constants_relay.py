#!/usr/bin/python3
import math
import time
import rospy
from pprint import pprint

from dynamic_reconfigure.server import Server
# from dynamic_reconfigure.client import Client
from tj2_networktables.cfg import SwerveConstantsConfig
from dynamic_reconfigure.encoding import decode_config, encode_config, encode_description, extract_params, get_tree, initial_config

from networktables import NetworkTables


class ConstantsRelay:
    def __init__(self):
        self.node_name = "constants_relay"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.nt_host = rospy.get_param("~nt_host", "10.0.88.2")
        
        NetworkTables.initialize(server=self.nt_host)
        time.sleep(2.0)  # wait for table to populate
        self.nt = NetworkTables.getTable("swerveLibrary")

        # self.nt.addEntryListener(self.ping_callback, key="pingResponse")

        starter_table = self.build_tables("configuration//modules")

        # print("starter_table")
        # pprint(starter_table)
        self.config_id_counter = 0

        self.namespace = "/tj2/swerve_constants"
        self.dyn_srv = Server(SwerveConstantsConfig, self.swerve_constants_callback, self.namespace)
        self.dyn_srv.update_configuration(self.to_config(starter_table))

        rospy.loginfo("%s init complete" % self.node_name)
    
    def to_config(self, table: dict):
        self.config_id_counter = 0
        config = {}
        self._to_config_recurse(table, config)
        config["name"] = "Default"
        config["state"] = True
        config["parent"] = 0
        config["id"] = self.config_id_counter
        config = {"groups": config}
        
        return config
    
    def _to_config_recurse(self, table, config):
        self.config_id_counter += 1
        config["groups"] = {}
        config["parameters"] = {}
        config["state"] = True
        config["parent"] = 0
        config["id"] = self.config_id_counter
        for key, value in table.items():
            if type(value) == dict:
                # if "groups" not in config:
                #     config["groups"] = {}
                self.config_id_counter += 1
                config["groups"][key] = {
                    "name": key,
                    "state": True,
                    "parent": config["id"],
                    "id": self.config_id_counter,
                }
                self._to_config_recurse(value, config["groups"][key])
            else:
                # if "parameters" not in config:
                #     config["parameters"] = {}
                config["parameters"][key] = value

    def build_tables(self, base_key):
        directory = {}
        self._build_tables_recurse(self.nt, base_key, directory)
        return directory
    
    def _build_tables_recurse(self, table, base_key, directory):
        sub_table = table.getSubTable(base_key)
        for sub_key in sub_table.getSubTables():
            full_key = base_key + "/" + sub_key
            directory[sub_key] = {}
            self._build_tables_recurse(table, full_key, directory[sub_key])
        for sub_key in sub_table.getKeys():
            full_key = base_key + "/" + sub_key
            value = table.getEntry(full_key)
            directory[sub_key] = value.get()

    def swerve_constants_callback(self, config, level):
        # rospy.loginfo("Level: %s" % level)
        # rospy.loginfo("Config: %s" % config)
        # print("config")
        pprint(config)
        encoded = encode_config(config)
        print(encoded)
        print(decode_config(encoded))
        return config

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = ConstantsRelay()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
