import os
import sys
import time
import glob
import rospkg
import argparse
from networktables import NetworkTables
from tj2_tools.networktables.backups import Backups


def find_recent_csv(directory: str) -> str:
    list_of_files = glob.glob(directory + "/*.csv")
    latest_file = max(list_of_files, key=os.path.getctime)
    return latest_file


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="apply NT backup", add_help=True)
    parser.add_argument("backup_path", help="Path to backup or folder to search for most recent backup")
    parser.add_argument("-s", "--delay", default=0.0, type=float, help="How many seconds to wait between writing values")
    parser.add_argument("-d", "--data", default=None, help="Path to tj2_data/data")
    parser.add_argument("-n", "--nt-host", default="10.0.88.2", help="NT host address")
    args = parser.parse_args()
    
    if args.data is None:
        rospack = rospkg.RosPack()
        data_package = "tj2_data"
        package_dir = rospack.get_path(data_package) + "/data"
    else:
        package_dir = args.data
    if len(package_dir) > 0:
        os.chdir(package_dir)

    if len(args.backup_path) == 0:
        backup_search = "preferences"
    else:
        backup_search = args.backup_path
    if os.path.isdir(backup_search):
        backup_path = find_recent_csv(backup_search)
    elif os.path.isfile(backup_search) and backup_search.endswith(".csv"):
        backup_path = backup_search
    else:
        raise FileNotFoundError(f"Failed to find a preferences file given '{backup_search}'")
    assert backup_path.endswith(".csv")

    table = Backups.read_backup(backup_path)
    
    NetworkTables.initialize(server=args.nt_host)
    nt = NetworkTables.getTable("")
    time.sleep(2.0)  # wait for NT to populate
    
    if Backups.write_full_table(nt, table, args.delay):
        print(f"Backup {backup_path} applied successfully!")
        time.sleep(1.0)  # wait for NT to populate
        sys.exit(0)
    else:
        print(f"Backup {backup_path} failed!!")
        sys.exit(1)
