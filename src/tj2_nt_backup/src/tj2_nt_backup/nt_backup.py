#!/usr/bin/env python3
import datetime
import os

import rospkg
import rospy
from networktables import NetworkTables
from std_msgs.msg import Time
from tj2_tools.networktables.backups import Backups


class NTBackup:
    def __init__(self) -> None:
        self.node_name = "nt_backup"
        rospy.init_node(self.node_name)

        self.robot = str(rospy.get_param("~robot", "diffyjr"))
        self.nt_host = str(rospy.get_param("~nt_host", "10.0.88.2"))
        self.name_format = str(rospy.get_param("~name_format", "preferences_%Y-%m-%d.csv"))

        self.rospack = rospkg.RosPack()
        self.data_package = "tj2_data"
        self.package_dir = self.rospack.get_path(self.data_package)
        self.default_backups_dir = self.package_dir + f"/data/preferences/{self.robot}"
        self.backups_dir = str(rospy.get_param("~backups_dir", self.default_backups_dir))

        NetworkTables.initialize(server=self.nt_host)
        self.root_key = "Preferences"
        self.nt = NetworkTables.getTable(self.root_key)
        rospy.sleep(2.0)  # Wait for NT to populate

        self.record_sub = rospy.Subscriber("backup_preferences", Time, self.record_callback, queue_size=1)

    def get_backup_path(self) -> str:
        now = datetime.datetime.now()
        filename = now.strftime(self.name_format)
        if not os.path.isdir(self.backups_dir):
            os.makedirs(self.backups_dir)
        path = os.path.join(self.backups_dir, filename)
        return path

    def record_callback(self, msg: Time) -> None:
        table = Backups.get_full_table(self.nt)
        path = self.get_backup_path()
        Backups.write_backup(path, table)
        rospy.loginfo(f"Wrote backup to {path}")

    def run(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    NTBackup().run()
