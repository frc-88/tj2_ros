import rospkg
import rospy
import datetime
from networktables import NetworkTables
from tj2_tools.networktables.backups import Backups


class NTBackup:
    def __init__(self) -> None:
        self.node_name = "nt_backup"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.nt_host = rospy.get_param("~nt_host", "10.0.88.2")
        self.startup_delay = rospy.get_param("~startup_delay", 0.0)
        self.backup_interval = rospy.get_param("~backup_interval", 0.0)  # 0.0 == once then exit
        self.name_format = rospy.get_param("~name_format", "preferences_%Y-%m-%d.csv")
        
        self.rospack = rospkg.RosPack()
        self.data_package = "tj2_data"
        self.package_dir = self.rospack.get_path(self.data_package)
        self.default_backups_dir = self.package_dir + "/data/preferences"
        self.backups_dir = rospy.get_param("~backups_dir", self.default_backups_dir)

        NetworkTables.initialize(server=self.nt_host)
        self.nt = NetworkTables.getTable("")
        self.root_key = "Preferences"

    def get_backup_path(self) -> str:
        now = datetime.datetime.now()
        filename = now.strftime(self.name_format)
        path = self.backups_dir + "/" + filename
        return path

    def run(self) -> None:
        rospy.sleep(self.startup_delay)
        
        if self.backup_interval == 0.0:
            rate = None
        else:
            rate = rospy.Rate(1.0 / self.backup_interval)
        
        while not rospy.is_shutdown():
            table = Backups.get_full_table(self.nt.getSubTable(self.root_key))
            path = self.get_backup_path()
            Backups.write_backup(path, table)
            rospy.loginfo(f"Wrote backup to {path}")
            
            if rate is None:
                break
            else:
                rate.sleep()


if __name__ == "__main__":
    node = NTBackup()
    node.run()
