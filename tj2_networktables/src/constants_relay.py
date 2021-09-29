#!/usr/bin/python3
import time
import rospy

from dynamic_reconfigure.server import Server
from tj2_networktables.cfg import AzimuthControllerConstantsConfig
from tj2_networktables.cfg import ModuleConstantsConfig
from tj2_networktables.cfg import MotorConstantsConfig
from tj2_networktables.cfg import SensorConstantsConfig
from tj2_networktables.cfg import WheelControllerConstantsConfig

from networktables import NetworkTables


class ConstantsRelay:
    def __init__(self):
        self.node_name = "constants_relay"
        rospy.init_node(
            self.node_name,
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.nt_host = rospy.get_param("~nt_host", "localhost")
        self.num_modules = rospy.get_param("~num_modules", 4)
        
        NetworkTables.initialize(server=self.nt_host)
        self.base_key = "swerveLibrary"
        self.nt = NetworkTables.getTable(self.base_key)
        self.config_key = "configuration"

        self.config_table = {}
        module_tables = {}
        for module_index in range(self.num_modules):
            module_table = {
                "": ModuleConstantsConfig,
                "azimuthController": AzimuthControllerConstantsConfig,
                "wheelController": WheelControllerConstantsConfig,
                "motors": {
                    "0": MotorConstantsConfig,
                    "1": MotorConstantsConfig,
                },
                "sensors": SensorConstantsConfig
            }
            module_tables[module_index] = module_table
        self.config_table["modules"] = module_tables

        self.namespace = "/tj2/swerve_config"
        self.path_table = self.decode_table(self.namespace, self.config_table)
        self.servers = self.build_servers(self.path_table)

        rospy.loginfo("%s init complete" % self.node_name)
    
    def _recurse_table(self, namespace, table, decoded):
        for key, value in table.items():
            sub_namespace = namespace + "/" + str(key)
            if type(value) == dict:
                self._recurse_table(sub_namespace, value, decoded)
            else:
                decoded[sub_namespace] = value
    
    def decode_table(self, namespace, table):
        decoded_table = {}
        self._recurse_table(namespace, table, decoded_table)
        return decoded_table
        
    def build_servers(self, table):
        servers = {}
        for path, config_cls in table.items():
            rospy.loginfo("Creating server for %s" % path)
            dyn_srv = Server(config_cls, lambda config, level, p=path: self.wrapped_callback(p, config, level), path)
            servers[path] = dyn_srv
        return servers

    def wrapped_callback(self, path, config, level):
        if level == -1:
            return config
        if not path.startswith(self.namespace):
            return config
        self.swerve_constants_callback(path, config)
        return config

    def swerve_constants_callback(self, path, config):
        sub_table = path[len(self.namespace) + 1:]  # exclude extra "/" in slice
        sub_table = "/".join((self.config_key, sub_table))

        rospy.loginfo("Setting %s to %s" % (sub_table, config))

        config_cls = self.path_table[path]
        for name, value_type in config_cls.type.items():
            entry_path = sub_table + "/" + str(name)
            if not self.nt.containsKey(entry_path):
                rospy.logdebug("\tKey %s does not exist. Skipping" % entry_path)
                continue

            entry = self.nt.getEntry(entry_path)
            value = config[name]
            if entry.get() == value:
                rospy.logdebug("\tKey %s is already the desired value. Skipping" % entry_path)
                continue

            rospy.logdebug("\tSet %s to %s" % (entry_path, value))
            
            if value_type == "double":
                entry.setDouble(value + 1)
            elif value_type == "int":
                entry.setNumber(value)
            elif value_type == "str":
                entry.setString(value)
            elif value_type == "bool":
                entry.setBoolean(value)
            else:
                rospy.logwarn("Invalid config type: %s (%s)" % (value_type, name))

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = ConstantsRelay()
    # try:
    node.run()

    # except rospy.ROSInterruptException:
    #     pass

    # finally:
    #     rospy.loginfo("Exiting %s node" % node.node_name)
