#!/usr/bin/python3
import os
import rospy

from tj2_networktables.msg import NTEntry


NT_TABLE = {}


def nt_callback(msg):
    NT_TABLE[msg.path] = msg.value

def main():
    rospy.init_node(
        "publish_match_info"
    )
    nt_sub = rospy.Subscriber("/tj2/smart_dashboard", NTEntry, nt_callback)

    while True:
        if rospy.is_shutdown():
            break
        print("---")
        for path, value in NT_TABLE.items():
            print("%s:\t%s" % (path, value))
        print()
        rospy.sleep(0.25)


if __name__ == '__main__':
    main()
