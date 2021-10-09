from pprint import pprint
import time
from networktables import NetworkTables


def build_tables(table, base_key):
    directory = {}
    tree = {base_key: {}}
    _build_tables_recurse(table, base_key, tree[base_key], directory)
    return tree, directory
    
def _build_tables_recurse(table, base_key, tree, directory):
    sub_table = table.getSubTable(base_key)
    for sub_key in sub_table.getSubTables():
        full_key = base_key + "/" + sub_key
        tree[sub_key] = {}
        _build_tables_recurse(table, full_key, tree[sub_key], directory)
    for sub_key in sub_table.getKeys():
        full_key = base_key + "/" + sub_key
        value = table.getEntry(full_key)
        tree[sub_key] = value.get()
        directory[full_key] = value.get()

def main():
    NetworkTables.initialize(server="10.0.88.2")
    time.sleep(2.0)

    tree, directory = build_tables(NetworkTables.getGlobalTable(), "swerveLibrary")

    # nt = NetworkTables.getGlobalTable()
    # print(nt.getSubTables())
    # print(nt.getEntry("configuration/modules/sensor/offset"))
    pprint(tree)

main()